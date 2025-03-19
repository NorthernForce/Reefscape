package frc.robot.subsystems.photonvision;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.photonvision.AprilTagCamera.CameraPoseEstimate;
import frc.robot.subsystems.photonvision.AprilTagCamera.RejectedPoseEstimate;

import static edu.wpi.first.units.Units.*;

/**
 * A subsystem that interfaces with PhotonVision cameras and pose estimators.
 * This class is responsible for updating the pose estimators with new camera
 * results and providing the latest pose estimates.
 */
public class PhotonVision extends SubsystemBase
{
    private final AprilTagCamera[] cameras;
    private final ArrayList<CameraPoseEstimate> poseEstimates;
    private final ArrayList<RejectedPoseEstimate> rejectedEstimates;
    private final Angle angularTolerance;
    private final LinearVelocity maxLinearVelocity;
    private final Distance closeDistance;
    private Time lastAcceptedPoseTimestamp;

    /**
     * Constructs a new PhotonVision subsystem with the given camera names, poses,
     * and field layout.
     * 
     * @param cameraNames The names of the cameras to use.
     * @param cameraPoses The poses of the cameras relative to the robot.
     * @param layout      The apriltag field layout to use.
     */
    public PhotonVision(AprilTagCamera[] cameras, LinearVelocity maxLinearVelocity, Angle angularTolerance,
            Distance closeDistance)
    {
        this.cameras = cameras;
        poseEstimates = new ArrayList<>();
        rejectedEstimates = new ArrayList<>();
        this.angularTolerance = angularTolerance;
        this.maxLinearVelocity = maxLinearVelocity;
        lastAcceptedPoseTimestamp = null;
        this.closeDistance = closeDistance;
    }

    private static boolean isTooFarFromLastPose(CameraPoseEstimate estimate, Pose2d referencePose, Time timestamp,
            Time lastAcceptedPoseTimestamp, LinearVelocity maxLinearVelocity)
    {
        Distance distance = Meters
                .of(estimate.pose().toPose2d().getTranslation().getDistance(referencePose.getTranslation()));
        Time time = timestamp.minus(lastAcceptedPoseTimestamp);
        LinearVelocity velocity = distance.div(time);
        return velocity.gt(maxLinearVelocity.times(2));
    }

    private static boolean isTooFarHeadingFromLastPose(CameraPoseEstimate estimate, Pose2d referencePose,
            Angle angularTolerance)
    {
        Rotation2d rotation = estimate.pose().toPose2d().getRotation();
        Rotation2d referenceRotation = referencePose.getRotation();
        Angle angularDistance = rotation.minus(referenceRotation).getMeasure();
        return angularDistance.abs(Degrees) > angularTolerance.in(Degrees);
    }

    public CameraPoseEstimate[] updatePoseEstimates(Time timestamp, Pose2d referencePose)
    {
        ArrayList<CameraPoseEstimate> estimates = new ArrayList<>();
        for (AprilTagCamera camera : cameras)
        {
            estimates.addAll(Set.of(camera.updatePoseEstimates(timestamp, referencePose)));
        }
        poseEstimates.clear();
        rejectedEstimates.clear();
        for (CameraPoseEstimate estimate : estimates)
        {
            if (lastAcceptedPoseTimestamp != null)
            {
                if (isTooFarFromLastPose(estimate, referencePose, timestamp, lastAcceptedPoseTimestamp,
                        maxLinearVelocity))
                {
                    rejectedEstimates.add(new RejectedPoseEstimate(estimate, "Too far from last pose"));
                    estimates.remove(estimate);
                } else if (isTooFarHeadingFromLastPose(estimate, referencePose, angularTolerance))
                {
                    rejectedEstimates.add(new RejectedPoseEstimate(estimate, "Too far heading from last pose"));
                    estimates.remove(estimate);
                }
            }
        }
        if (estimates.size() > 2)
        {
            // Filter out estimates that are too far from the others
            for (CameraPoseEstimate estimate : estimates)
            {
                List<CameraPoseEstimate> closeEstimates = new ArrayList<>();
                for (CameraPoseEstimate other : estimates)
                {
                    if (estimate.pose().toPose2d().getTranslation()
                            .getDistance(other.pose().toPose2d().getTranslation()) <= closeDistance.in(Meters))
                    {
                        closeEstimates.add(other);
                    }
                }
                if (closeEstimates.size() > 1)
                {
                    poseEstimates.add(estimate);
                } else
                {
                    rejectedEstimates.add(new RejectedPoseEstimate(estimate, "Not enough close estimates"));
                }
            }
        } else
        {
            poseEstimates.addAll(estimates);
        }
        if (!poseEstimates.isEmpty())
        {
            lastAcceptedPoseTimestamp = timestamp;
        }
        return getPoseEstimates();
    }

    @AutoLogOutput
    public CameraPoseEstimate[] getPoseEstimates()
    {
        return poseEstimates.toArray(new CameraPoseEstimate[0]);
    }

    @AutoLogOutput
    public RejectedPoseEstimate[] getRejectedEstimates()
    {
        return rejectedEstimates.toArray(new RejectedPoseEstimate[0]);
    }
}
