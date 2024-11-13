package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

public class CameraManagerIOPhotonVision implements CameraManagerIO
{
	private final ArrayList<PhotonVisionCamera> photonVisionCameras;
	private ArrayList<EstimatedRobotPose> estimatedPoses;

	/**
	 * Class to manage any number PhotonVisionCamera objects
	 * 
	 * @param photonVisionCamera The PhotonVisionCamera objects to be managed
	 */
	public CameraManagerIOPhotonVision(PhotonVisionCamera... photonVisionCameras)
	{
		estimatedPoses = new ArrayList<>();
		this.photonVisionCameras = new ArrayList<>();
		for (PhotonVisionCamera cam : photonVisionCameras)
		{
			this.photonVisionCameras.add(cam);
		}
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
	public ArrayList<EstimatedRobotPose> getEstimatedRobotPoses(Pose2d prevEstimatedRobotPose)
	{
		ArrayList<EstimatedRobotPose> poses = new ArrayList<>();
		for (PhotonVisionCamera cam : photonVisionCameras)
		{
			poses.add(cam.getEstimatedRobotPose(prevEstimatedRobotPose));
		}
		estimatedPoses = poses;
		return poses;
	}
}
