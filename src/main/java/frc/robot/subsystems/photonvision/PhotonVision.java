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

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
        OUT_OF_BOUNDS, TARGET_OUTSIDE_USABLE_AREA, ROBOT_ANGLE_TOO_LARGE, DISTANCE_TOO_FAR
    }

    public static record RejectedPoseEstimate(RejectionReason reason, PoseEstimate pose) {
    }

    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] poseEstimators;
    private final ArrayList<PoseEstimate> poseEstimates;
    private final ArrayList<RejectedPoseEstimate> rejectedEstimates;
    private final double maxYCoordinate;
    private Pose2d lastKnownRobotPose;
    private double lastKnownVisionPoseTimestamp;
    private final AngularVelocity maxAngularVelocity;
    private final LinearVelocity maxLinearVelocity;
    private final AprilTagFieldLayout layout;
    private final double cameraWidth;
    private final Alert[] alerts;

    /**
     * Constructs a new PhotonVision subsystem with the given camera names, poses,
     * and field layout.
     * 
     * @param cameraNames The names of the cameras to use.
     * @param cameraPoses The poses of the cameras relative to the robot.
     * @param layout      The apriltag field layout to use.
     */
    public PhotonVision(String[] cameraNames, Transform3d[] cameraPoses, AprilTagFieldLayout layout,
            double maxYCoordinate, AngularVelocity maxAngularVelocity, LinearVelocity maxLinearVelocity,
            double cameraWidth)
    {
        cameras = new PhotonCamera[cameraNames.length];
        poseEstimators = new PhotonPoseEstimator[cameraNames.length];
        alerts = new Alert[cameraNames.length];
        for (int i = 0; i < cameraNames.length; i++)
        {
            cameras[i] = new PhotonCamera(cameraNames[i]);
            poseEstimators[i] = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    cameraPoses[i]);
            poseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
            alerts[i] = new Alert("PhotonVision Camera " + cameraNames[i] + " disconnected", AlertType.kError);
        }
        poseEstimates = new ArrayList<>();
        rejectedEstimates = new ArrayList<>();
        this.maxYCoordinate = maxYCoordinate;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxLinearVelocity = maxLinearVelocity;
        this.layout = layout;
        this.cameraWidth = cameraWidth;
    }

    private double getYCoordinate(List<TargetCorner> corners)
    {
        var y = (corners.get(0).y + corners.get(1).y + corners.get(2).y + corners.get(3).y) / 4;
        y -= cameraWidth / 2;
        return y;
    }

    public void setLastKnownRobotPose(Pose2d pose)
    {
        for (PhotonPoseEstimator estimator : poseEstimators)
        {
            estimator.setLastPose(pose);
        }
        lastKnownRobotPose = pose;
    }

    @SuppressWarnings("unused")
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

    @SuppressWarnings("unused")
    private boolean testRobotRotation(EstimatedRobotPose pose)
    {
        if (lastKnownRobotPose == null)
        {
            return true;
        }
        double maxDegreesDifference = maxAngularVelocity.in(DegreesPerSecond) * 0.02 * 5;
        double difference = pose.estimatedPose.toPose2d().getRotation().getDegrees()
                - lastKnownRobotPose.getRotation().getDegrees();
        return Math.abs(difference) < maxDegreesDifference;
    }

    @SuppressWarnings("unused")
    private boolean testRobotDistance(EstimatedRobotPose pose)
    {
        if (lastKnownRobotPose == null)
        {
            return true;
        }
        double maxDistanceDifference = maxLinearVelocity.in(MetersPerSecond) * 0.02 * 3;
        double difference = pose.estimatedPose.toPose2d().getTranslation()
                .getDistance(lastKnownRobotPose.getTranslation());
        return Math.abs(difference) < maxDistanceDifference;
    }

    @SuppressWarnings("unused")
    private boolean testEstimateTime(EstimatedRobotPose pose)
    {
        return Math.abs(pose.timestampSeconds - lastKnownVisionPoseTimestamp) < 1;
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
        for (int i = 0; i < cameras.length; i++)
        {
            alerts[i].set(!cameras[i].isConnected());
        }
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
                // if (!testYCoordinate(result))
                // {
                // valid = false;
                // reason = RejectionReason.TARGET_OUTSIDE_USABLE_AREA;
                // }
                // if (!testRobotRotation(opt.get()))
                // {
                // valid = false;
                // reason = RejectionReason.ROBOT_ANGLE_TOO_LARGE;
                // }
                // if (!testRobotDistance(opt.get()))
                // {
                // valid = false;
                // reason = RejectionReason.DISTANCE_TOO_FAR;
                // }
                if (!testWithinField(opt.get()))
                {
                    valid = false;
                    reason = RejectionReason.OUT_OF_BOUNDS;
                }
                if (valid)
                {
                    poseEstimates.add(new PoseEstimate(opt.get().estimatedPose.toPose2d(), opt.get().timestampSeconds));
                    lastKnownVisionPoseTimestamp = Math.max(opt.get().timestampSeconds, lastKnownVisionPoseTimestamp);
                } else
                {
                    rejectedEstimates.add(new RejectedPoseEstimate(reason,
                            new PoseEstimate(opt.get().estimatedPose.toPose2d(), opt.get().timestampSeconds)));
                }
            }
        }
    }

    public void updateWithHeading(Rotation2d newHeading)
    {
        for (var poseEstimator : poseEstimators)
        {
            poseEstimator.addHeadingData(Utils.getCurrentTimeSeconds(), newHeading);
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
