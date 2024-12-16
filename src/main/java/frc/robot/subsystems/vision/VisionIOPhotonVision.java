package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonVision implements VisionIO
{
	protected final PhotonCamera camera;
	protected final Transform3d robotToCamera;

	/**
	 * Class to manage any number PhotonVisionCamera objects
	 * 
	 * @param photonVisionCamera The PhotonVisionCamera objects to be managed
	 */
	public VisionIOPhotonVision(String name, Transform3d robotToCamera)
	{
		camera = new PhotonCamera(name);
		this.robotToCamera = robotToCamera;
	}

	@Override
	public void updateInputs(VisionIOInputs inputs)
	{
		Set<Short> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();
		for (var result : camera.getAllUnreadResults())
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
}
