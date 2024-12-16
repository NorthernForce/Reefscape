package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.vision.VisionConstants.*;

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

public class Vision extends SubsystemBase
{
	private final VisionConsumer consumer;
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;

	/**
	 * Class to manage any number PhotonVisionCamera objects
	 * 
	 * @param photonVisionCamera The PhotonVisionCamera objects to be managed
	 */
	public Vision(VisionConsumer consumer, VisionIO... io)
	{
		this.consumer = consumer;
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
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			for (int tagId : inputs[cameraIndex].tagIds)
			{
				var tagPose = aprilTagLayout.getTagPose(tagId);
				if (tagPose.isPresent())
				{
					tagPoses.add(tagPose.get());
				}
			}

			for (var observation : inputs[cameraIndex].poseObservations)
			{
				boolean rejectPose = observation.tagCount() == 0
						|| (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
						|| Math.abs(observation.pose().getZ()) > maxZError || observation.pose().getX() < 0.0
						|| observation.pose().getX() > aprilTagLayout.getFieldLength()
						|| observation.pose().getY() < 0.0
						|| observation.pose().getY() > aprilTagLayout.getFieldWidth();

				robotPoses.add(observation.pose());
				if (rejectPose)
				{
					robotPosesRejected.add(observation.pose());
				} else
				{
					robotPosesAccepted.add(observation.pose());
				}

				if (rejectPose)
				{
					continue;
				}

				double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
				double linearStdDev = linearStdDevBaseline * stdDevFactor;
				double angularStdDev = angularStdDevBaseline * stdDevFactor;
				if (observation.type() == PoseObservationType.MEGATAG_2)
				{
					linearStdDev *= linearStdDevMegatag2Factor;
					angularStdDev *= angularStdDevMegatag2Factor;
				}
				if (cameraIndex < cameraStdDevFactor.length)
				{
					linearStdDev *= cameraStdDevFactor[cameraIndex];
					angularStdDev *= cameraStdDevFactor[cameraIndex];
				}

				consumer.accept(observation.pose().toPose2d(), observation.timestamp(),
						VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			}

			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
					tagPoses.toArray(new Pose3d[tagPoses.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
					robotPoses.toArray(new Pose3d[robotPoses.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
					robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
					robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
		}

	}

	@FunctionalInterface
	public static interface VisionConsumer
	{
		public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds,
				Matrix<N3, N1> visionMeasurementStdDevs);
	}
}
