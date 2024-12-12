package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class PhotonVisionCamera extends SubsystemBase
{
	private final PhotonCamera photonCamera;
	private final PhotonPoseEstimator photonPoseEstimator;

	/**
	 * Wrapper class for a PhotonCamera and PhotonPoseEstimator
	 * 
	 * @param aprilTagFieldLayout The layout of the apriltags on the field
	 * @param robotToCam          Transform between the robot origin and the camera
	 * @param cameraNum           Used to give each camera a unique name
	 */
	public PhotonVisionCamera(AprilTagFieldLayout aprilTagFieldLayout, Transform3d robotToCam, int cameraNum)
	{
		photonCamera = new PhotonCamera("Global_Shutter_Camera (" + cameraNum + ")");
		photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO,
				robotToCam);
		photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}

	/**
	 * Wrapper class for a PhotonCamera and PhotonPoseEstimator
	 * 
	 * @param aprilTagFieldLayout The layout of the apriltags on the field
	 * @param poseStrategy        Strategy for PhotonPoseEstimator to use
	 * @param robotToCam          Transform between the robot origin and the camera
	 */
	public PhotonVisionCamera(AprilTagFieldLayout aprilTagFieldLayout, Transform3d robotToCam,
			PoseStrategy poseStrategy)
	{
		photonCamera = new PhotonCamera("photonvision");
		photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, robotToCam);
		photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}

	/**
	 * Gets estimated robot pose according to PhotonPoseEstimator
	 * 
	 * @param prevEstimatedRobotPose Used as a reference to make sure
	 *                               PhotonPoseEstimator doesn't do something funky
	 * @return Pose2d if present, else null
	 */
	public EstimatedRobotPose getEstimatedRobotPose(Pose2d prevEstimatedRobotPose)
	{
		photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		List<PhotonPipelineResult> camResults = photonCamera.getAllUnreadResults();
		Optional<EstimatedRobotPose> estimatedPose = null;
		for (PhotonPipelineResult camResult : camResults)
		{
			estimatedPose = photonPoseEstimator.update(camResult);
		}
		if (estimatedPose.isPresent())
		{
			return estimatedPose.get();
		} else
		{
			return null;
		}
	}

	public boolean isConnected()
	{
		return photonCamera.isConnected();
	}
}
