package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;

public class PhotonManager
{
    private final PhotonVisionCamera photonVisionCamera;

    /**
     * Class to manage any number PhotonVisionCamera objects
     * 
     * @param photonVisionCamera The PhotonVisionCamera objects to be managed
     */
    public PhotonManager(PhotonVisionCamera photonVisionCamera)
    {
        this.photonVisionCamera = photonVisionCamera;
    }

    /**
     * 
     * @param prevEstimatedRobotPose
     * @return
     */
    public ArrayList<Pose2d> getEstimatedRobotPose(Pose2d prevEstimatedRobotPose)
    {
        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(photonVisionCamera.getEstimatedRobotPose(prevEstimatedRobotPose).estimatedPose.toPose2d());
        return poses;
    }
}
