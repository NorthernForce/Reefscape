package frc.subsystems;
import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class VisionSystem {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    public VisionSystem(Transform3d robotToCam) throws IOException{
        camera = new PhotonCamera("photonvision");
        aprilTagFieldLayout = new AprilTagFieldLayout("");
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose){
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
