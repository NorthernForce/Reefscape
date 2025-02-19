package frc.robot.zippy;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.leds.LEDS;
import frc.robot.subsystems.leds.LedsIOCANdle;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.dashboard.DashboardIOFWC;
import frc.robot.subsystems.dashboard.reefscape.ReefDisplayIOSwing;
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.util.AutoRoutine;
import frc.robot.zippy.constants.ZippyConstants;
import frc.robot.zippy.constants.ZippyTunerConstants;
import frc.robot.zippy.oi.ZippyDriverOI;
import frc.robot.zippy.oi.ZippyProgrammerOI;

public class ZippyContainer implements NFRRobotContainer
{
    private final PhoenixCommandDrive drive;
    private final Supplier<Alliance> allianceSupplier = () -> DriverStation.getAlliance().orElse(Alliance.Red);
    private Alliance alliance = allianceSupplier.get();
    private final Dashboard dashboard;
    private final Command testCommand;

    private final LEDS leds = new LEDS(
            new LedsIOCANdle(ZippyConstants.LedConstants.CanID, ZippyConstants.LedConstants.ledInputs));

    public ZippyContainer()
    {
        dashboard = new Dashboard(new ReefDisplayIOSwing("ReefscapeDisplay"), new DashboardIOFWC());
        drive = new PhoenixCommandDrive(ZippyTunerConstants.DrivetrainConstants,
                ZippyConstants.DrivetrainConstants.MAX_SPEED, ZippyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                ZippyTunerConstants.FrontLeft, ZippyTunerConstants.FrontRight, ZippyTunerConstants.BackLeft,
                ZippyTunerConstants.BackRight);
        drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(alliance));
        dashboard.addDefaultAutoRoutine("Do Nothing", new AutoRoutine(new InstantCommand(), new Translation2d[]
        { new Translation2d(), new Translation2d() }, new Pose2d()));
        testCommand = Commands.parallel(drive.getIdleCommand());
        dashboard.setResetEncodersCommand(drive.runOnce(this::resetDriveEncoders).ignoringDisable(true));
        leds.setDefaultCommand(leds.getRainbowAnimation().ignoringDisable(true));
        SmartDashboard.putString("continer", "zippy");

    }

    public PhoenixCommandDrive getDrive()
    {
        return drive;
    }

    public LEDS getLEDs()
    {
        return leds;
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
    }

    @Override
    public void autonomousInit()
    {
        drive.resetPose(
                FieldConstants.convertPoseByAlliance(dashboard.getRoutine().startPose(), FieldConstants.getAlliance()));
    }

    @Override
    public void teleopInit()
    {
        dashboard.setTeleopStage();
    }

    @Override
    public void disabledInit()
    {
        if (testCommand.isScheduled())
        {
            testCommand.cancel();
        }
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
        testCommand.schedule();
        dashboard.setSettingsStage();
    }

}
