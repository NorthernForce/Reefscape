package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class PhotonManager {
    private final PhotonVisionCamera photonVisionCamera;

    //TODO: Implement ServeDrivePoseEstimator and allow for multiple cameras

    public PhotonManager(PhotonVisionCamera photonVisionCamera){
        this.photonVisionCamera = photonVisionCamera;
    }

    public Pose2d getEstimatedRobotPose(Pose2d prevEstimatedRobotPose){
        return photonVisionCamera.getEstimatedRobotPose(prevEstimatedRobotPose);
    }
}
