package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class CameraManagerIOPhotonVisionSim implements CameraManagerIO
{
	private final PhotonVisionCameraSim[] photonVisionCameras;
	private boolean[] connected;
	private EstimatedRobotPose[] estimatedPoses;
	private int numberOfCameras;
	private VisionSystemSim photonSim;

	/**
	 * Class to manage any number PhotonVisionCamera objects
	 * 
	 * @param photonVisionCamera The PhotonVisionCamera objects to be managed
	 */
	public CameraManagerIOPhotonVisionSim(PhotonVisionCameraSim... photonVisionCameras)
	{
		numberOfCameras = photonVisionCameras.length;
		connected = new boolean[numberOfCameras];
		estimatedPoses = new EstimatedRobotPose[numberOfCameras];
		this.photonVisionCameras = photonVisionCameras;
		photonSim = new VisionSystemSim("main");
		photonSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo));
		for (PhotonVisionCameraSim cam : photonVisionCameras)
		{
			photonSim.addCamera(cam.getCameraSim(), cam.getRobotToCamera());
		}
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
			PhotonVisionCameraSim cam = photonVisionCameras[i];
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
			PhotonVisionCameraSim simCam = photonVisionCameras[i];
			PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
					AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), PoseStrategy.MULTI_TAG_PNP_ON_RIO,
					simCam.getRobotToCamera());
			PhotonPipelineResult camResult = simCam.getCameraSim().getCamera().getAllUnreadResults().get(0);
			Optional<EstimatedRobotPose> estimatedPose = null;
			estimatedPose = poseEstimator.update(camResult);
			if (estimatedPose.isPresent())
			{
				poses[i] = estimatedPose.get();
			} else
			{
				poses[i] = null;
			}
		}
		estimatedPoses = poses;
		return poses;
	}

	@Override
	public void updateSimulationWithPose(Pose2d pose)
	{
		photonSim.update(pose);
	}
}
