package frc.robot.zippy;

import java.util.Map;
import java.util.Set;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.northernforce.util.NFRRobotContainer;

import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefPositions.ReefSide;
import frc.robot.ralph.constants.RalphConstants;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.dashboard.DashboardIOFWC;
import frc.robot.subsystems.dashboard.reefscape.ReefDisplayIOSwing;
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.util.NFRAutoRoutine;
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
    private Field2d field = null;
    private AutoRoutine hi = null;

    public ZippyContainer()
    {
        dashboard = new Dashboard(new ReefDisplayIOSwing("ReefscapeDisplay"), new DashboardIOFWC());
        drive = new PhoenixCommandDrive(ZippyTunerConstants.DrivetrainConstants,
                ZippyConstants.DrivetrainConstants.MAX_SPEED, ZippyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                ZippyConstants.PathplannerConstants.linearPIDConstants,
                ZippyConstants.PathplannerConstants.angularPIDConstants, Meters.of(0),
                ZippyConstants.AutoConstants.xPID, ZippyConstants.AutoConstants.yPID, ZippyConstants.AutoConstants.rPID,
                ZippyConstants.DrivetrainConstants.SWERVE_MODULE_OFFSETS, ZippyTunerConstants.FrontLeft,
                ZippyTunerConstants.FrontRight, ZippyTunerConstants.BackLeft, ZippyTunerConstants.BackRight);
        drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(alliance));
        vision = new PhotonVision(ZippyConstants.VisionConstants.cameraNames(),
                ZippyConstants.VisionConstants.cameraTransforms(), ZippyConstants.VisionConstants.APRILTAG_LAYOUT,
                ZippyConstants.VisionConstants.MAX_Y_COORDINATE, ZippyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                ZippyConstants.DrivetrainConstants.MAX_LINEAR_SPEED, ZippyConstants.VisionConstants.CAMERA_WIDTH);
        LoggedPowerDistribution.getInstance(40, ModuleType.kRev);
        dashboard.addDefaultAutoRoutine("Do Nothing", new NFRAutoRoutine(new InstantCommand(), new Translation2d[]
        { new Translation2d(), new Translation2d() }, () -> new Pose2d()));

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
        field.setRobotPose(drive.getPose());
        dashboard.updatePose(drive.getPose());
        vision.setLastKnownRobotPose(drive.getPose());
        for (var poseEstimate : vision.getPoseEstimates())
        {
            drive.addVisionMeasurement(poseEstimate.pose(), Utils.fpgaToCurrentTime(poseEstimate.timestamp()));
        }
    }

    @Override
    public void autonomousInit()
    {
        drive.resetPose(FieldConstants.convertPoseByAlliance(dashboard.getRoutine().startPose().get(),
                FieldConstants.getAlliance()));

        NamedCommands.registerCommand("test", Commands.runOnce(() -> System.out.println("it works!")));
        var routine = factory.newRoutine("test1");
        var traj = routine.trajectory("testPath");
        routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
        hi = routine;
        // hicmd = factory.trajectoryCmd("testPath");
        // RobotModeTriggers.autonomous().whileTrue(hicmd).whileTrue(Commands.run(() ->
        // System.out.println("")));
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
        return dashboard.getRoutine().command();
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

    public Command getGoToReefPoseCommandLeft()
    {
        Pose2d currentPose = drive.getPose();
        ReefSide closestTranslation = FieldConstants.ReefPositions.SIDES[0];
        for (ReefSide pose : FieldConstants.ReefPositions.SIDES)
        {
            if (pose.getDistanceFromCenter(currentPose).in(Meters) < closestTranslation
                    .getDistanceFromCenter(currentPose).in(Meters) || closestTranslation == null)
            {
                closestTranslation = pose;
            }
        }

        Pose2d finalClosestTranslation = new Pose2d(closestTranslation.left().getX(), closestTranslation.left().getY(),
                closestTranslation.left().getRotation());
        double angle = Math.toRadians(finalClosestTranslation.getRotation().getDegrees());
        double deltaX = -10 * Math.cos(angle + 3.0 * Math.PI / 2.0) / 39.37; // 10 inches to meters
        double deltaY = -10 * Math.sin(angle + 3.0 * Math.PI / 2.0) / 39.37; // 10 inches to meters
        Pose2d adjustePose2d = new Pose2d(finalClosestTranslation.getX() + deltaX,
                finalClosestTranslation.getY() + deltaY, finalClosestTranslation.getRotation());

        return Commands.defer(() -> getDrive().driveToPose(
                FieldConstants.convertPoseByAlliance(adjustePose2d, FieldConstants.getAlliance()),
                RalphConstants.PathplannerConstants.MAX_VELOCITY, RalphConstants.PathplannerConstants.MAX_ACCELERATION,
                RalphConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                RalphConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION), Set.of());
    }

    public Command getGoToReefPoseCommandCenter()
    {
        Pose2d currentPose = drive.getPose();
        ReefSide closestTranslation = FieldConstants.ReefPositions.SIDES[0];
        for (ReefSide pose : FieldConstants.ReefPositions.SIDES)
        {
            if (pose.getDistanceFromCenter(currentPose).in(Meters) < closestTranslation
                    .getDistanceFromCenter(currentPose).in(Meters) || closestTranslation == null)
            {
                closestTranslation = pose;
            }
        }

        Pose2d finalClosestTranslation = new Pose2d(closestTranslation.center().getX(),
                closestTranslation.center().getY(), closestTranslation.center().getRotation());
        double angle = Math.toRadians(finalClosestTranslation.getRotation().getDegrees());
        double deltaX = -10 * Math.cos(angle + 3.0 * Math.PI / 2.0) / 39.37; // 10 inches to meters
        double deltaY = -10 * Math.sin(angle + 3.0 * Math.PI / 2.0) / 39.37; // 10 inches to meters
        Pose2d adjustePose2d = new Pose2d(finalClosestTranslation.getX() + deltaX,
                finalClosestTranslation.getY() + deltaY, finalClosestTranslation.getRotation());

        return Commands.defer(() -> getDrive().driveToPose(
                FieldConstants.convertPoseByAlliance(adjustePose2d, FieldConstants.getAlliance()),
                RalphConstants.PathplannerConstants.MAX_VELOCITY, RalphConstants.PathplannerConstants.MAX_ACCELERATION,
                RalphConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                RalphConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION), Set.of());
    }

    public Command getGoToReefPoseCommandRight()
    {
        Pose2d currentPose = drive.getPose();
        ReefSide closestTranslation = FieldConstants.ReefPositions.SIDES[0];
        for (ReefSide pose : FieldConstants.ReefPositions.SIDES)
        {
            if (pose.getDistanceFromCenter(currentPose).in(Meters) < closestTranslation
                    .getDistanceFromCenter(currentPose).in(Meters) || closestTranslation == null)
            {
                closestTranslation = pose;
            }
        }

        Pose2d finalClosestTranslation = new Pose2d(closestTranslation.right().getX(),
                closestTranslation.right().getY(), closestTranslation.right().getRotation());
        double angle = Math.toRadians(finalClosestTranslation.getRotation().getDegrees());
        double deltaX = -10 * Math.cos(angle + 3.0 * Math.PI / 2.0) / 39.37; // 10 inches to meters
        double deltaY = -10 * Math.sin(angle + 3.0 * Math.PI / 2.0) / 39.37; // 10 inches to meters
        Pose2d adjustePose2d = new Pose2d(finalClosestTranslation.getX() + deltaX,
                finalClosestTranslation.getY() + deltaY, finalClosestTranslation.getRotation());

        return Commands.defer(() -> getDrive().driveToPose(
                FieldConstants.convertPoseByAlliance(adjustePose2d, FieldConstants.getAlliance()),
                RalphConstants.PathplannerConstants.MAX_VELOCITY, RalphConstants.PathplannerConstants.MAX_ACCELERATION,
                RalphConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                RalphConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION), Set.of());
    }

}
