package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.VisionConstants;

public class Vision extends SubsystemBase
{
	private final VisionConsumer consumer;
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;
	private final VisionConstants visionConstants;

	/**
	 * Class to manage any number PhotonVisionCamera objects
	 * 
	 * @param photonVisionCamera The PhotonVisionCamera objects to be managed
	 */
	public Vision(VisionConsumer consumer, VisionConstants visionConstants, VisionIO... io)
	{
		this.consumer = consumer;
		this.visionConstants = visionConstants;
		this.io = io;

		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < inputs.length; i++)
		{
			inputs[i] = new VisionIOInputsAutoLogged();
		}

		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++)
		{
			disconnectedAlerts[i] = new Alert("Vision camera " + Integer.toString(i) + " is disconnected.",
					AlertType.kWarning);
		}
	}

	public Rotation2d getTargetX(int cameraIndex)
	{
		return inputs[cameraIndex].latestTargetObservation.tx();
	}

	@Override
	public void periodic()
	{
		// Process all camera inputs
		for (int i = 0; i < io.length; i++)
		{
			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
		}

		List<Pose3d> allTagPoses = new LinkedList<>();
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();

		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++)
		{
			// Check if the camera is disconnected
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			// Adds tag poses if they exist
			for (int tagId : inputs[cameraIndex].tagIds)
			{
				var tagPose = visionConstants.aprilTagLayout().getTagPose(tagId);
				if (tagPose.isPresent())
				{
					tagPoses.add(tagPose.get());
					allTagPoses.add(tagPose.get());
				}
			}

			for (var observation : inputs[cameraIndex].poseObservations)
			{
				// All the reasons why a pose might be rejected: no tags, too ambiguous, outside
				// field boundaries
				boolean rejectPose = observation.tagCount() == 0
						|| (observation.tagCount() == 1 && observation.ambiguity() > visionConstants.maxAmbiguity())
						|| Math.abs(observation.pose().getZ()) > visionConstants.maxZError()
						|| observation.pose().getX() < 0.0
						|| observation.pose().getX() > visionConstants.aprilTagLayout().getFieldLength()
						|| observation.pose().getY() < 0.0
						|| observation.pose().getY() > visionConstants.aprilTagLayout().getFieldWidth();

				robotPoses.add(observation.pose());
				allRobotPoses.add(observation.pose());

				if (rejectPose)
				{
					robotPosesRejected.add(observation.pose());
					allRobotPosesRejected.add(observation.pose());
				} else
				{
					robotPosesAccepted.add(observation.pose());
					allRobotPosesAccepted.add(observation.pose());
				}

				if (rejectPose)
				{
					continue;
				}

				// Find the standard deviation of the pose
				double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
				double linearStdDev = visionConstants.linearStdDevBaseline() * stdDevFactor;
				double angularStdDev = visionConstants.angularStdDevBaseline() * stdDevFactor;
				if (observation.type() == PoseObservationType.MEGATAG_2)
				{
					linearStdDev *= visionConstants.linearStdDevMegatag2Factor();
					angularStdDev *= visionConstants.angularStdDevMegatag2Factor();
				}
				if (cameraIndex < visionConstants.cameraStdDevFactor().length)
				{
					linearStdDev *= visionConstants.cameraStdDevFactor()[cameraIndex];
					angularStdDev *= visionConstants.cameraStdDevFactor()[cameraIndex];
				}

				// Give the pose to the drive subsystem
				consumer.accept(observation.pose().toPose2d(), observation.timestamp(),
						VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			}

			// Log the poses with AdvantageKit
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
					tagPoses.toArray(new Pose3d[tagPoses.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
					robotPoses.toArray(new Pose3d[robotPoses.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
					robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
					robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
		}

		Logger.recordOutput("Vision/AllTagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
		Logger.recordOutput("Vision/AllRobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
		Logger.recordOutput("Vision/AllRobotPosesAccepted",
				allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
		Logger.recordOutput("Vision/AllRobotPosesRejected",
				allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
	}

	@FunctionalInterface
	public static interface VisionConsumer
	{
		public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds,
				Matrix<N3, N1> visionMeasurementStdDevs);
	}
}
