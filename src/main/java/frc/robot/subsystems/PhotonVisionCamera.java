package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class PhotonVisionCamera extends SubsystemBase
{
	private final PhotonCamera photonCamera;
	private final PhotonPoseEstimator photonPoseEstimator;
	private final AprilTagFieldLayout aprilTagFieldLayout;

	public PhotonVisionCamera(Transform3d robotToCam)
	{
		aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
		photonCamera = new PhotonCamera("photonvision");
		photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
				photonCamera, robotToCam);
	}

	public Pose2d getEstimatedRobotPose(Pose2d prevEstimatedRobotPose)
	{
		photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		return photonPoseEstimator.update().get().estimatedPose.toPose2d();
	}
}
