package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class PhotonVisionCamera extends SubsystemBase
{
	protected final PhotonCamera photonCamera;
	protected final Transform3d robotToCamera;

	/**
	 * Wrapper class for a PhotonCamera and PhotonPoseEstimator
	 * 
	 * @param aprilTagFieldLayout The layout of the apriltags on the field
	 * @param robotToCamera       Transform between the robot origin and the camera
	 * @param cameraNum           Used to give each camera a unique name
	 */
	public PhotonVisionCamera(String name, Transform3d robotToCamera)
	{
		photonCamera = new PhotonCamera(name);
		this.robotToCamera = robotToCamera;
	}

	/**
	 * Gets estimated robot pose according to PhotonPoseEstimator
	 * 
	 * @param prevEstimatedRobotPose Used as a reference to make sure
	 *                               PhotonPoseEstimator doesn't do something funky
	 * @return Pose2d if present, else null
	 */
	public void updateInputs(VisionIOInputs inputs, int camIndex)
	{
		Set<Short> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();
		for (var result : photonCamera.getAllUnreadResults())
		{
			if (result.hasTargets())
			{
				inputs.latestTargetObservation = new TargetObservation(
						Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
						Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
			} else
			{
				inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
			}

			if (result.multitagResult.isPresent())
			{
				var multitagResult = result.multitagResult.get();

				Transform3d fieldToCamera = multitagResult.estimatedPose.best;
				Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
				Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

				double totalTagDistance = 0.0;
				for (var target : result.getTargets())
				{
					totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
				}

				tagIds.addAll(multitagResult.fiducialIDsUsed);

				poseObservations.add(new PoseObservation(result.getTimestampSeconds(), robotPose,
						multitagResult.estimatedPose.ambiguity, multitagResult.fiducialIDsUsed.size(),
						totalTagDistance / result.targets.size(), PoseObservationType.PHOTONVISION));
			}
		}

		inputs.poseObservations = new PoseObservation[poseObservations.size()];
		for (int i = 0; i < poseObservations.size(); i++)
		{
			inputs.poseObservations[i] = poseObservations.get(i);
		}

		inputs.tagIds = new int[tagIds.size()];
		int i = 0;
		for (int id : tagIds)
		{
			inputs.tagIds[i++] = id;
		}
	}

	public boolean isConnected()
	{
		return photonCamera.isConnected();
	}
}
