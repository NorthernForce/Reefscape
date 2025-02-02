package frc.robot.subsystems.photonvision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

/**
 * A subsystem that interfaces with PhotonVision cameras and pose estimators.
 * This class is responsible for updating the pose estimators with new camera
 * results and providing the latest pose estimates.
 */
public class PhotonVision extends SubsystemBase
{
    public static enum RejectionReason
    {
        OUT_OF_BOUNDS, TARGET_OUTSIDE_USABLE_AREA, ROBOT_ANGLE_TOO_LARGE
    }

    public static record RejectedPoseEstimate(RejectionReason reason, PoseEstimate pose) {
    }

    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] poseEstimators;
    private final ArrayList<PoseEstimate> poseEstimates;
    private final ArrayList<RejectedPoseEstimate> rejectedEstimates;
    private final double maxYCoordinate;
    private Rotation2d lastKnownRobotRotation;
    private final AngularVelocity maxAngularVelocity;
    private final AprilTagFieldLayout layout;
    private final double cameraWidth;

    /**
     * Constructs a new PhotonVision subsystem with the given camera names, poses,
     * and field layout.
     * 
     * @param cameraNames The names of the cameras to use.
     * @param cameraPoses The poses of the cameras relative to the robot.
     * @param layout      The apriltag field layout to use.
     */
    public PhotonVision(String[] cameraNames, Transform3d[] cameraPoses, AprilTagFieldLayout layout,
            double maxYCoordinate, AngularVelocity maxAngularVelocity, double cameraWidth)
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
        rejectedEstimates = new ArrayList<>();
        this.maxYCoordinate = maxYCoordinate;
        this.maxAngularVelocity = maxAngularVelocity;
        this.layout = layout;
        this.cameraWidth = cameraWidth;
    }

    private double getYCoordinate(List<TargetCorner> corners)
    {
        var y = (corners.get(0).y + corners.get(1).y + corners.get(2).y + corners.get(3).y) / 4;
        y -= cameraWidth / 2;
        return y;
    }

    public void setLastKnownRobotRotation(Rotation2d rotation)
    {
        lastKnownRobotRotation = rotation;
    }

    private boolean testYCoordinate(PhotonPipelineResult result)
    {
        for (var target : result.getTargets())
        {
            if (Math.abs(getYCoordinate(target.getDetectedCorners())) > maxYCoordinate)
            {
                return false;
            }
        }
        return true;
    }

    private boolean testRobotRotation(EstimatedRobotPose pose)
    {
        if (lastKnownRobotRotation == null)
        {
            return true;
        }
        double maxDegreesDifference = maxAngularVelocity.in(DegreesPerSecond) * 0.02 * 5;
        double difference = pose.estimatedPose.toPose2d().getRotation().getDegrees()
                - lastKnownRobotRotation.getDegrees();
        return Math.abs(difference) < maxDegreesDifference;
    }

    private boolean testWithinField(EstimatedRobotPose pose)
    {
        return pose.estimatedPose.toPose2d().getTranslation().getX() > 0
                && pose.estimatedPose.toPose2d().getTranslation().getX() < layout.getFieldLength()
                && pose.estimatedPose.toPose2d().getTranslation().getY() > 0
                && pose.estimatedPose.toPose2d().getTranslation().getY() < layout.getFieldWidth();
    }

    @Override
    public void periodic()
    {
        poseEstimates.clear();
        rejectedEstimates.clear();
        for (int i = 0; i < cameras.length; i++)
        {
            for (var result : cameras[i].getAllUnreadResults())
            {
                var opt = poseEstimators[i].update(result);
                if (opt.isEmpty())
                {
                    continue;
                }
                boolean valid = true;
                RejectionReason reason = null;
                if (!testYCoordinate(result))
                {
                    valid = false;
                    reason = RejectionReason.TARGET_OUTSIDE_USABLE_AREA;
                }
                // if (!testRobotRotation(opt.get()))
                // {
                // valid = false;
                // reason = RejectionReason.ROBOT_ANGLE_TOO_LARGE;
                // }
                if (!testWithinField(opt.get()))
                {
                    valid = false;
                    reason = RejectionReason.OUT_OF_BOUNDS;
                }
                if (valid)
                {
                    poseEstimates.add(new PoseEstimate(opt.get().estimatedPose.toPose2d(), opt.get().timestampSeconds));
                } else
                {
                    rejectedEstimates.add(new RejectedPoseEstimate(reason,
                            new PoseEstimate(opt.get().estimatedPose.toPose2d(), opt.get().timestampSeconds)));
                }
            }
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
        return poseEstimates.toArray(poses);
    }

    @AutoLogOutput
    public RejectedPoseEstimate[] getRejectedPoseEstimates()
    {
        RejectedPoseEstimate[] poses = new RejectedPoseEstimate[rejectedEstimates.size()];
        return rejectedEstimates.toArray(poses);
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
