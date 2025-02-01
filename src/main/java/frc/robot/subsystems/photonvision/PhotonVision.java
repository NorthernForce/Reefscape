package frc.robot.subsystems.photonvision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that interfaces with PhotonVision cameras and pose estimators.
 * This class is responsible for updating the pose estimators with new camera
 * results and providing the latest pose estimates.
 */
public class PhotonVision extends SubsystemBase
{
    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] poseEstimators;
    private final ArrayList<EstimatedRobotPose> poseEstimates;
    private final double maxYCoordinate;

    /**
     * Constructs a new PhotonVision subsystem with the given camera names, poses,
     * and field layout.
     * 
     * @param cameraNames The names of the cameras to use.
     * @param cameraPoses The poses of the cameras relative to the robot.
     * @param layout      The apriltag field layout to use.
     */
    public PhotonVision(String[] cameraNames, Transform3d[] cameraPoses, AprilTagFieldLayout layout,
            double maxYCoordinate)
    {
        cameras = new PhotonCamera[cameraNames.length];
        poseEstimators = new PhotonPoseEstimator[cameraNames.length];
        for (int i = 0; i < cameraNames.length; i++)
        {
            cameras[i] = new PhotonCamera(cameraNames[i]);
            poseEstimators[i] = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    cameraPoses[i]);
        }
        poseEstimates = new ArrayList<>();
        this.maxYCoordinate = maxYCoordinate;
    }

    private static double getYCoordinate(List<TargetCorner> corners)
    {
        return (corners.get(0).y + corners.get(1).y + corners.get(2).y + corners.get(3).y) / 4;
    }

    private void processCamera(int idx)
    {
        for (var result : cameras[idx].getAllUnreadResults())
        {
            for (var target : result.getTargets())
            {
                if (Math.abs(getYCoordinate(target.getDetectedCorners())) > maxYCoordinate)
                {
                    System.out.println("Target y out of range: " + getYCoordinate(target.getDetectedCorners()));
                    return;
                }
            }
            var opt = poseEstimators[idx].update(result);
            if (opt.isPresent())
            {
                poseEstimates.add(opt.get());
            }
        }
    }

    @Override
    public void periodic()
    {
        poseEstimates.clear();
        for (int i = 0; i < cameras.length; i++)
        {
            processCamera(i);
        }
    }

    /**
     * Returns the latest pose estimates from the PhotonVision cameras.
     * 
     * @return The latest pose estimates.
     */
    @AutoLogOutput
    public PoseEstimate[] getPoseEstimates()
    {
        PoseEstimate[] poses = new PoseEstimate[poseEstimates.size()];
        for (int i = 0; i < poses.length; i++)
        {
            poses[i] = new PoseEstimate(poseEstimates.get(i).estimatedPose.toPose2d(),
                    poseEstimates.get(i).timestampSeconds);
        }
        return poses;
    }

    @AutoLogOutput
    public boolean[] getConnectedStatus()
    {
        boolean[] connected = new boolean[cameras.length];
        for (int i = 0; i < cameras.length; i++)
        {
            connected[i] = cameras[i].isConnected();
        }
        return connected;
    }

    /**
     * A record that represents a pose estimate from a PhotonVision camera. This
     * record contains the pose estimate and the timestamp of the estimate.
     * 
     * @param pose      The pose estimate.
     * @param timestamp The timestamp of the estimate.
     */
    public static record PoseEstimate(Pose2d pose, double timestamp) {
    }
}
