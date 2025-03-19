package frc.robot.zippy;

import java.util.Map;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.northernforce.util.NFRRobotContainer;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
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
    private Field2d field = null;
    private AutoRoutine hi = null;

    public ZippyContainer()
    {
        dashboard = new Dashboard(new ReefDisplayIOSwing("ReefscapeDisplay"), new DashboardIOFWC());
        drive = new PhoenixCommandDrive(ZippyTunerConstants.DrivetrainConstants,
                ZippyConstants.DrivetrainConstants.MAX_SPEED, ZippyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                ZippyConstants.DrivetrainConstants.MAX_ACCELERATION,
                ZippyConstants.DrivetrainConstants.MAX_ANGULAR_ACCELERATION,
                ZippyConstants.DrivetrainConstants.TRANSITION_SPEED,
                ZippyConstants.DrivetrainConstants.TRANSITION_DISTANCE,
                ZippyConstants.PathplannerConstants.linearPIDConstants,
                ZippyConstants.PathplannerConstants.angularPIDConstants,
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

    public Command getBackupAuto()
    {
        return drive.backup(Seconds.of(1), 0.5);
    }

}
