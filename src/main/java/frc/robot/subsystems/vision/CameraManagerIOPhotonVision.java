package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

public class CameraManagerIOPhotonVision implements CameraManagerIO
{
	private final PhotonVisionCamera[] photonVisionCameras;
	private boolean[] connected;
	private EstimatedRobotPose[] estimatedPoses;
	private int numberOfCameras;

	/**
	 * Class to manage any number PhotonVisionCamera objects
	 * 
	 * @param photonVisionCamera The PhotonVisionCamera objects to be managed
	 */
	public CameraManagerIOPhotonVision(PhotonVisionCamera... photonVisionCameras)
	{
		numberOfCameras = photonVisionCameras.length;
		connected = new boolean[numberOfCameras];
		estimatedPoses = new EstimatedRobotPose[numberOfCameras];
		this.photonVisionCameras = photonVisionCameras;
	}

	@Override
	public void updateInputs(CameraManagerIOInputs inputs)
	{
		// inputs.connectedCameras = connected;
		for (int i = 0; i < numberOfCameras; i++)
		{
			EstimatedRobotPose pose = estimatedPoses[i];
			inputs.poses[i] = pose.estimatedPose.toPose2d();
			inputs.timestamps[i] = pose.timestampSeconds;
		}
	}

	public boolean[] getConnectedCameras()
	{
		boolean[] connectedCameras = new boolean[numberOfCameras];
		for (int i = 0; i < connected.length; i++)
		{
			PhotonVisionCamera cam = photonVisionCameras[i];
			connectedCameras[i] = cam.isConnected();
		}
		connected = connectedCameras;
		return connected;
	}

	/**
	 * 
	 * @param prevEstimatedRobotPose
	 * @return ArrayList of estimated poses from all the PhotonVisionCameras
	 */
	public EstimatedRobotPose[] getEstimatedRobotPoses(Pose2d prevEstimatedRobotPose)
	{
		EstimatedRobotPose[] poses = new EstimatedRobotPose[numberOfCameras];
		for (int i = 0; i < poses.length; i++)
		{
			PhotonVisionCamera cam = photonVisionCameras[i];
			poses[i] = cam.getEstimatedRobotPose(prevEstimatedRobotPose);
		}
		estimatedPoses = poses;
		return poses;
	}
}
