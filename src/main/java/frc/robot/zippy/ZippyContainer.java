package frc.robot.zippy;

import java.util.Map;
import java.util.Set;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.northernforce.util.NFRRobotContainer;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefPositions.ReefSide;
import frc.robot.ralph.constants.RalphConstants.DrivetrainConstants;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.dashboard.DashboardIOFWC;
import frc.robot.subsystems.dashboard.reefscape.ReefDisplayIOSwing;
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.subsystems.photonvision.PhotonVision;
import frc.robot.zippy.constants.ZippyConstants;
import frc.robot.zippy.constants.ZippyTunerConstants;
import frc.robot.zippy.oi.ZippyDriverOI;
import frc.robot.zippy.oi.ZippyProgrammerOI;

public class ZippyContainer implements NFRRobotContainer
{
    private final PhoenixCommandDrive drive;
    private final Supplier<Alliance> allianceSupplier = () -> DriverStation.getAlliance().orElse(Alliance.Red);
    private Alliance alliance = allianceSupplier.get();
    private final PhotonVision vision;
    private final Dashboard dashboard;
    private AutoFactory factory = null;
    // private Field2d field = null;
    private AutoRoutine hi = null;

    public ZippyContainer()
    {
        dashboard = new Dashboard(new ReefDisplayIOSwing("ReefscapeDisplay"), new DashboardIOFWC());
        drive = new PhoenixCommandDrive(ZippyTunerConstants.DrivetrainConstants,
                ZippyConstants.DrivetrainConstants.MAX_SPEED, ZippyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                ZippyConstants.PathplannerConstants.linearPIDConstants,
                ZippyConstants.PathplannerConstants.angularPIDConstants, Meters.of(0),
                ZippyConstants.AutoConstants.xPID, ZippyConstants.AutoConstants.yPID, ZippyConstants.AutoConstants.rPID,
                ZippyConstants.AutoConstants.kP, ZippyConstants.AutoConstants.kI, ZippyConstants.AutoConstants.kD,
                ZippyConstants.AutoConstants.postP, ZippyConstants.AutoConstants.postI,
                ZippyConstants.AutoConstants.postD, ZippyConstants.AutoConstants.kConstraints,
                ZippyConstants.AutoConstants.kPRotation, ZippyConstants.AutoConstants.rotationContinuous,
                ZippyConstants.AutoConstants.totalAngle, ZippyConstants.AutoConstants.totalDistance,
                ZippyConstants.DrivetrainConstants.SWERVE_MODULE_OFFSETS, ZippyTunerConstants.FrontLeft,
                ZippyTunerConstants.FrontRight, ZippyTunerConstants.BackLeft, ZippyTunerConstants.BackRight);
        drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(alliance));
        vision = new PhotonVision(ZippyConstants.VisionConstants.cameraNames(),
                ZippyConstants.VisionConstants.cameraTransforms(), ZippyConstants.VisionConstants.APRILTAG_LAYOUT,
                ZippyConstants.VisionConstants.MAX_Y_COORDINATE, ZippyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                ZippyConstants.DrivetrainConstants.MAX_LINEAR_SPEED, ZippyConstants.VisionConstants.CAMERA_WIDTH);
        LoggedPowerDistribution.getInstance(40, ModuleType.kRev);

        dashboard.setResetEncodersCommand(drive.runOnce(this::resetDriveEncoders).ignoringDisable(true));
    }

    public PhoenixCommandDrive getDrive()
    {
        return drive;
    }

    public Dashboard getDashboard()
    {
        return dashboard;
    }

    @Override
    public void bindDriverOI()
    {
        new ZippyDriverOI().bindOI(this);
    }

    @Override
    public void bindProgrammerOI()
    {
        new ZippyProgrammerOI().bindOI(this);
    }

    @Override
    public void periodic()
    {
        if (alliance != allianceSupplier.get())
        {
            alliance = allianceSupplier.get();
            drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(allianceSupplier.get()));
        }
        // field.setRobotPose(drive.getPose());
        dashboard.updatePose(drive.getPose());
        vision.setLastKnownRobotPose(drive.getPose());
        for (var poseEstimate : vision.getPoseEstimates())
        {
            drive.addVisionMeasurement(poseEstimate.pose(), Utils.fpgaToCurrentTime(poseEstimate.timestamp()));
        }
        SmartDashboard.updateValues();
    }

    @Override
    public void autonomousInit()
    {

    }

    @Override
    public void autonomousPeriodic()
    {
        System.out.println("running auto...");
        hi.poll();
    }

    public Map<String, Supplier<AutoRoutine>> getAutonomousCommands()
    {
        return Map.of("nothing", () -> factory.newRoutine("nothing"));
    }

    @Override
    public void teleopInit()
    {
        dashboard.setTeleopStage();
    }

    @Override
    public void disabledInit()
    {
        dashboard.setAutoStage();
    }

    @Override
    public Command getAutonomousCommand()
    {
        return dashboard.getRoutine();
    }

    private void resetDriveEncoders()
    {
        final var offsets = drive.resetEncoderAngles(new Angle[]
        { Degrees.of(0), Degrees.of(0), Degrees.of(0), Degrees.of(0) });
        Preferences.setDouble("kSwerveOffsetFrontLeft", offsets[0].in(Rotations));
        Preferences.setDouble("kSwerveOffsetFrontRight", offsets[1].in(Rotations));
        Preferences.setDouble("kSwerveOffsetBackLeft", offsets[2].in(Rotations));
        Preferences.setDouble("kSwerveOffsetBackRight", offsets[3].in(Rotations));
    }

    @Override
    public void testInit()
    {
        dashboard.setSettingsStage();
    }

    public Pose2d getTargetPoseLeft(Pose2d currentPose)
    {
        ReefSide closestTranslation = FieldConstants.ReefPositions.SIDES[0];
        for (ReefSide pose : FieldConstants.ReefPositions.SIDES)
        {
            if (pose.getDistanceFromLeft(currentPose).in(Meters) < closestTranslation.getDistanceFromCenter(currentPose)
                    .in(Meters))
            {
                closestTranslation = pose;
            }
        }
        double angleRadians = closestTranslation.left().getRotation().getRadians() + 3 * Math.PI / 2;
        double xOffset = Inches.of(Math.cos(angleRadians) * -10).in(Meters);
        double yOffset = Inches.of(Math.sin(angleRadians) * -10).in(Meters);
        Pose2d finalTranslatedPose2d = new Pose2d(closestTranslation.left().getTranslation().getX() + xOffset,
                closestTranslation.left().getTranslation().getY() + yOffset, closestTranslation.left().getRotation());
        return FieldConstants.convertPoseByAlliance(finalTranslatedPose2d, FieldConstants.getAlliance());
    }

    public Pose2d getTargetPoseCenter(Pose2d currentPose)
    {
        ReefSide closestTranslation = FieldConstants.ReefPositions.SIDES[0];
        for (ReefSide pose : FieldConstants.ReefPositions.SIDES)
        {
            if (pose.getDistanceFromCenter(currentPose).in(Meters) < closestTranslation
                    .getDistanceFromCenter(currentPose).in(Meters))
            {
                closestTranslation = pose;
            }
        }

        Pose2d finalTranslatedPose2d = new Pose2d(closestTranslation.center().getTranslation().getX(),
                closestTranslation.center().getTranslation().getY(), closestTranslation.center().getRotation());
        return FieldConstants.convertPoseByAlliance(finalTranslatedPose2d, FieldConstants.getAlliance());
    }

    public Pose2d getTargetPoseRight(Pose2d currentPose)
    {
        ReefSide closestTranslation = FieldConstants.ReefPositions.SIDES[0];
        for (ReefSide pose : FieldConstants.ReefPositions.SIDES)
        {
            if (pose.getDistanceFromCenter(currentPose).in(Meters) < closestTranslation
                    .getDistanceFromCenter(currentPose).in(Meters) || closestTranslation == null)
            {
                closestTranslation = pose;
            }
        }
        double angleRadians = closestTranslation.right().getRotation().getRadians() + 3 * Math.PI / 2;
        double xOffset = Inches.of(Math.cos(angleRadians) * -10).in(Meters);
        double yOffset = Inches.of(Math.sin(angleRadians) * -10).in(Meters);
        Pose2d finalTranslatedPose2d = new Pose2d(closestTranslation.right().getTranslation().getX() + xOffset,
                closestTranslation.right().getTranslation().getY() + yOffset, closestTranslation.right().getRotation());
        return FieldConstants.convertPoseByAlliance(finalTranslatedPose2d, FieldConstants.getAlliance());
    }

    public Command getGoToReefPoseCommandLeft()
    {
        return Commands.defer(() -> getDrive().driveToAccurate(getTargetPoseLeft(getDrive().getPose()),
                DrivetrainConstants.MAX_LINEAR_SPEED, DrivetrainConstants.MAX_ACCELERATION,
                DrivetrainConstants.MAX_ANGULAR_SPEED, DrivetrainConstants.MAX_ANGULAR_ACCELERATION,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KP, DrivetrainConstants.CLOSE_TRANSLATION_PP_KI,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KD, DrivetrainConstants.CLOSE_ROTATION_PP_KP,
                DrivetrainConstants.CLOSE_ROTATION_PP_KI, DrivetrainConstants.CLOSE_ROTATION_PP_KD), Set.of());
    }

    public Command getGoToReefPoseCommandRight()
    {
        return Commands.defer(() -> getDrive().driveToAccurate(getTargetPoseRight(getDrive().getPose()),
                DrivetrainConstants.MAX_LINEAR_SPEED, DrivetrainConstants.MAX_ACCELERATION,
                DrivetrainConstants.MAX_ANGULAR_SPEED, DrivetrainConstants.MAX_ANGULAR_ACCELERATION,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KP, DrivetrainConstants.CLOSE_TRANSLATION_PP_KI,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KD, DrivetrainConstants.CLOSE_ROTATION_PP_KP,
                DrivetrainConstants.CLOSE_ROTATION_PP_KI, DrivetrainConstants.CLOSE_ROTATION_PP_KD), Set.of());
    }

    public Command getGoToReefPoseCommandCenter()
    {
        return Commands.defer(() -> getDrive().driveToAccurate(getTargetPoseCenter(getDrive().getPose()),
                DrivetrainConstants.MAX_LINEAR_SPEED, DrivetrainConstants.MAX_ACCELERATION,
                DrivetrainConstants.MAX_ANGULAR_SPEED, DrivetrainConstants.MAX_ANGULAR_ACCELERATION,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KP, DrivetrainConstants.CLOSE_TRANSLATION_PP_KI,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KD, DrivetrainConstants.CLOSE_ROTATION_PP_KP,
                DrivetrainConstants.CLOSE_ROTATION_PP_KI, DrivetrainConstants.CLOSE_ROTATION_PP_KD), Set.of());
    }

    public Pose2d getClosestCoralStation(Pose2d pose)
    {
        if (FieldConstants.CoralStations.LEFT.getTranslation()
                .getDistance(pose.getTranslation()) < FieldConstants.CoralStations.RIGHT.getTranslation()
                        .getDistance(pose.getTranslation()))
        {
            return FieldConstants.CoralStations.LEFT;
        } else
        {
            return FieldConstants.CoralStations.RIGHT;
        }
    }

    public Command getGoToProcessorCommand()
    {
        return Commands.defer(() -> getDrive().driveToAccurate(FieldConstants.ProcessorStations.PROCESSOR_STATION,
                DrivetrainConstants.MAX_LINEAR_SPEED, DrivetrainConstants.MAX_ACCELERATION,
                DrivetrainConstants.MAX_ANGULAR_SPEED, DrivetrainConstants.MAX_ANGULAR_ACCELERATION,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KP, DrivetrainConstants.CLOSE_TRANSLATION_PP_KI,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KD, DrivetrainConstants.CLOSE_ROTATION_PP_KP,
                DrivetrainConstants.CLOSE_ROTATION_PP_KI, DrivetrainConstants.CLOSE_ROTATION_PP_KD), Set.of());
    }

    public Command driveToCoralStation()
    {
        return Commands.defer(() -> getDrive().driveToAccurate(getClosestCoralStation(getDrive().getPose()),
                DrivetrainConstants.MAX_LINEAR_SPEED, DrivetrainConstants.MAX_ACCELERATION,
                DrivetrainConstants.MAX_ANGULAR_SPEED, DrivetrainConstants.MAX_ANGULAR_ACCELERATION,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KP, DrivetrainConstants.CLOSE_TRANSLATION_PP_KI,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KD, DrivetrainConstants.CLOSE_ROTATION_PP_KP,
                DrivetrainConstants.CLOSE_ROTATION_PP_KI, DrivetrainConstants.CLOSE_ROTATION_PP_KD), Set.of());
    }
}
