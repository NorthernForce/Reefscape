package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class PhotonVisionCamera extends SubsystemBase
{
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    public PhotonVisionCamera(AprilTagFieldLayout aprilTagFieldLayout, PoseStrategy poseStrategy,
            Transform3d robotToCam)
    {
        photonCamera = new PhotonCamera("photonvision");
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, photonCamera, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Pose2d getEstimatedRobotPose(Pose2d prevEstimatedRobotPose)
    {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update();
        if (estimatedPose.isPresent())
        {
            return estimatedPose.get().estimatedPose.toPose2d();
        } else
        {
            return null;
        }
    }
}
