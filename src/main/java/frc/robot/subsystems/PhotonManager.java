package frc.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

public class PhotonManager
{
    private final ArrayList<PhotonVisionCamera> photonVisionCameras;

    /**
     * Class to manage any number PhotonVisionCamera objects
     * 
     * @param photonVisionCamera The PhotonVisionCamera objects to be managed
     */
    public PhotonManager(PhotonVisionCamera... photonVisionCameras)
    {
        this.photonVisionCameras = new ArrayList<>();
        for (PhotonVisionCamera cam : photonVisionCameras)
        {
            this.photonVisionCameras.add(cam);
        }
    }

    /**
     * 
     * @param prevEstimatedRobotPose
     * @return
     */
    public ArrayList<EstimatedRobotPose> getEstimatedRobotPoses(Pose2d prevEstimatedRobotPose)
    {
        ArrayList<EstimatedRobotPose> poses = new ArrayList<>();
        for (PhotonVisionCamera cam : photonVisionCameras)
        {
            poses.add(cam.getEstimatedRobotPose(prevEstimatedRobotPose));
        }
        return poses;
    }
}
