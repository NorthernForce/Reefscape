package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

public class CameraManagerIOPhotonVision implements CameraManagerIO
{
	private final PhotonVisionCamera[] photonVisionCameras;
	private EstimatedRobotPose[] estimatedPoses;

	/**
	 * Class to manage any number PhotonVisionCamera objects
	 * 
	 * @param photonVisionCamera The PhotonVisionCamera objects to be managed
	 */
	public CameraManagerIOPhotonVision(PhotonVisionCamera... photonVisionCameras)
	{
		estimatedPoses = new EstimatedRobotPose[photonVisionCameras.length];
		this.photonVisionCameras = photonVisionCameras;
	}

	@Override
	public void updateInputs(CameraManagerIOInputs inputs)
	{
		inputs.estimatedPoses = estimatedPoses;
	}

	/**
	 * 
	 * @param prevEstimatedRobotPose
	 * @return ArrayList of estimated poses from all the PhotonVisionCameras
	 */
	public EstimatedRobotPose[] getEstimatedRobotPoses(Pose2d prevEstimatedRobotPose)
	{
		EstimatedRobotPose[] poses = new EstimatedRobotPose[photonVisionCameras.length];
		for (int i = 0; i < poses.length; i++)
		{
            PhotonVisionCamera cam = photonVisionCameras[i];
			poses[i] = cam.getEstimatedRobotPose(prevEstimatedRobotPose);
		}
		estimatedPoses = poses;
		return poses;
	}
}
