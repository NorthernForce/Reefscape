package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class PhotonVisionCameraSim extends SubsystemBase
{
	private final PhotonCamera photonCamera;
	private final PhotonCameraSim photonCameraSim;
	private final SimCameraProperties simCameraProperties;
	private final Transform3d robotToCam;

	/**
	 * Wrapper class for a PhotonCamera and PhotonPoseEstimator
	 * 
	 * @param aprilTagFieldLayout The layout of the apriltags on the field
	 * @param robotToCam          Transform between the robot origin and the camera
	 * @param cameraNum           Used to give each camera a unique name
	 */
	public PhotonVisionCameraSim(AprilTagFieldLayout aprilTagFieldLayout, Transform3d robotToCam, int cameraNum)
	{
		simCameraProperties = new SimCameraProperties();
		simCameraProperties.setCalibration(320, 240, Rotation2d.fromDegrees(70));
		simCameraProperties.setCalibError(0.25, 0.08);
		simCameraProperties.setFPS(5);
		simCameraProperties.setAvgLatencyMs(35);
		simCameraProperties.setLatencyStdDevMs(5);
		this.robotToCam = robotToCam;

		photonCamera = new PhotonCamera("Global_Shutter_Camera (" + cameraNum + ")");
		photonCameraSim = new PhotonCameraSim(photonCamera, simCameraProperties);
	}

	/**
	 * Gets estimated robot pose according to PhotonPoseEstimator
	 * 
	 * @param prevEstimatedRobotPose Used as a reference to make sure
	 *                               PhotonPoseEstimator doesn't do something funky
	 * @return Pose2d if present, else null
	 */
	public PhotonCameraSim getCameraSim()
	{
		return photonCameraSim;
	}

	public Transform3d getRobotToCamera()
	{
		return robotToCam;
	}

	public boolean isConnected()
	{
		return photonCamera.isConnected();
	}
}
