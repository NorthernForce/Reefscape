package frc.robot.subsystems.photonvision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagCamera extends SubsystemBase
{
    public record CameraPoseEstimate(Time timestamp, Pose3d pose) {
    }

    public record RejectedPoseEstimate(CameraPoseEstimate estimate, String reason) {
    }

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final ArrayList<CameraPoseEstimate> poseEstimates;
    private final ArrayList<RejectedPoseEstimate> rejectedEstimates;
    private final Alert cameraNotPresentAlert;
    private final Distance maxZ;
    private final double maxAmbiguity;
    public final String name;

    public AprilTagCamera(String name, AprilTagFieldLayout field, Transform3d robotToCamera, Distance maxZ,
            double maxAmbiguity)
    {
        setName(name);
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_RIO, robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        poseEstimates = new ArrayList<>();
        rejectedEstimates = new ArrayList<>();
        cameraNotPresentAlert = new Alert("Camera " + name + " not present", AlertType.kError);
        this.maxZ = maxZ;
        this.maxAmbiguity = maxAmbiguity;
        this.name = name;
    }

    public void updateReferencePose(Time timestamp, Pose2d pose)
    {
        poseEstimator.setReferencePose(new Pose3d(pose));
        poseEstimator.addHeadingData(timestamp.in(Seconds), pose.getRotation());
    }

    private static boolean isOnField(Pose2d pose, AprilTagFieldLayout field)
    {
        return pose.getY() >= 0 && pose.getY() <= field.getFieldWidth() && pose.getX() >= 0
                && pose.getX() <= field.getFieldLength();
    }

    private static boolean isRobotTooHigh(Pose3d pose, Distance maxZ)
    {
        return pose.getTranslation().getZ() > maxZ.in(Meters) || pose.getTranslation().getZ() < -maxZ.in(Meters);
    }

    private static boolean isOnOppositeAlliance(Pose2d pose, AprilTagFieldLayout field)
    {
        return (pose.getY() >= field.getFieldWidth() / 2
                && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue)
                || (pose.getY() < field.getFieldWidth() / 2
                        && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red);
    }

    private static boolean isPoseAmbiguityTooHigh(EstimatedRobotPose pose, double maxAmbiguity)
    {
        for (var tag : pose.targetsUsed)
        {
            if (tag.poseAmbiguity > maxAmbiguity)
            {
                return true;
            }
        }
        return false;
    }

    public CameraPoseEstimate[] updatePoseEstimates(Time timestamp, Pose2d referencePose)
    {
        poseEstimates.clear();
        rejectedEstimates.clear();
        updateReferencePose(timestamp, referencePose);
        for (var result : camera.getAllUnreadResults())
        {
            var poseEstimate = poseEstimator.update(result);
            if (poseEstimate.isPresent())
            {
                var pose = poseEstimate.get();
                if (!isOnField(pose.estimatedPose.toPose2d(), poseEstimator.getFieldTags()))
                {
                    rejectedEstimates.add(new RejectedPoseEstimate(
                            new CameraPoseEstimate(timestamp, pose.estimatedPose), "Off field"));
                    continue;
                }
                if (isRobotTooHigh(pose.estimatedPose, maxZ))
                {
                    rejectedEstimates.add(new RejectedPoseEstimate(
                            new CameraPoseEstimate(timestamp, pose.estimatedPose), "Robot too high"));
                    continue;
                }
                if (isOnOppositeAlliance(pose.estimatedPose.toPose2d(), poseEstimator.getFieldTags()))
                {
                    rejectedEstimates.add(new RejectedPoseEstimate(
                            new CameraPoseEstimate(timestamp, pose.estimatedPose), "On opposite alliance"));
                    continue;
                }
                if (isPoseAmbiguityTooHigh(pose, maxAmbiguity))
                {
                    rejectedEstimates.add(new RejectedPoseEstimate(
                            new CameraPoseEstimate(timestamp, pose.estimatedPose), "Pose ambiguity too high"));
                    continue;
                }
                poseEstimates.add(new CameraPoseEstimate(timestamp, pose.estimatedPose));
            }
        }
        return poseEstimates.toArray(new CameraPoseEstimate[0]);
    }

    @AutoLogOutput(key = "{name}/PoseEstimates")
    public CameraPoseEstimate[] getPoseEstimates()
    {
        return poseEstimates.toArray(new CameraPoseEstimate[0]);
    }

    @AutoLogOutput(key = "{name}/RejectedEstimates")
    public RejectedPoseEstimate[] getRejectedEstimates()
    {
        return rejectedEstimates.toArray(new RejectedPoseEstimate[0]);
    }

    @AutoLogOutput(key = "{name}/CameraConnected")
    public boolean isCameraConnected()
    {
        return camera.isConnected();
    }

    @Override
    public void periodic()
    {
        cameraNotPresentAlert.set(!camera.isConnected());
    }
}
