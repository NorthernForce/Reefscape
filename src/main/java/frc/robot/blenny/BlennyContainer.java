package frc.robot.blenny;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.blenny.constants.BlennyConstants;
import frc.robot.blenny.constants.BlennyTunerConstants;
import frc.robot.blenny.oi.BlennyDriverOI;
import frc.robot.blenny.oi.BlennyProgrammerOI;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.dashboard.DashboardIOFWC;
import frc.robot.subsystems.dashboard.reefscape.ReefDisplayIOSwing;
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIOTalonFXS;
import frc.robot.subsystems.rollers.sensor.RollersSensorIOAnalog;
import frc.robot.subsystems.photonvision.PhotonVision;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.elevator.brake.BrakeIO;
import frc.robot.subsystems.superstructure.elevator.sensor.ElevatorSensorIO;
import frc.robot.subsystems.superstructure.elevator.sensor.ElevatorSensorIOLimitSwitch;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristIO;
import frc.robot.subsystems.superstructure.wrist.WristIOTalonFX;
import frc.robot.util.AutoRoutine;

/**
 * 2025 Competition Robot Container. Name is still a work in progress and will
 * likely change. Blenny is a type of fish. It is also the name of a submarine
 * that was sunk to create an artificial reef.
 */
public class BlennyContainer implements NFRRobotContainer
{
    private final PhoenixCommandDrive drive;
    private final Rollers rollers;
    private final Superstructure superstructure;
    private final PhotonVision vision;
    private final Supplier<Alliance> allianceSupplier = () -> DriverStation.getAlliance().orElse(Alliance.Red);
    private Alliance alliance = allianceSupplier.get();
    private final Climber climber;
    private final Dashboard dashboard;
    private boolean inAlgaeState = false;
    private Alert hallDisconnectedAlert;

    /**
     * Create a new BlennyContainer
     */
    public BlennyContainer()
    {
        hallDisconnectedAlert = new Alert("Hall sensor disconnected", AlertType.kError);
        drive = new PhoenixCommandDrive(BlennyTunerConstants.DrivetrainConstants,
                BlennyConstants.DrivetrainConstants.MAX_SPEED, BlennyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                BlennyConstants.PathplannerConstants.linearPIDConstants,
                BlennyConstants.PathplannerConstants.angularPIDConstants);

        rollers = new Rollers(
                new RollersIOTalonFXS(BlennyConstants.RollersConstants.ROLLER_MOTOR_LEFT_ID,
                        BlennyConstants.RollersConstants.ROLLER_MOTOR_RIGHT_ID,
                        BlennyConstants.RollersConstants.ROLLER_MOTORS_INVERTED),
                new RollersSensorIOAnalog(BlennyConstants.RollersConstants.SensorConstants.ANALOG_ALGAE,
                        BlennyConstants.RollersConstants.SensorConstants.ALGAE_MAX_DISTANCE),
                new RollersSensorIOAnalog(BlennyConstants.RollersConstants.SensorConstants.ANALOG_CORAL,
                        BlennyConstants.RollersConstants.SensorConstants.CORAL_MAX_DISTANCE));

        vision = new PhotonVision(BlennyConstants.VisionConstants.cameraNames(),
                BlennyConstants.VisionConstants.cameraTransforms(), BlennyConstants.VisionConstants.APRILTAG_LAYOUT,
                BlennyConstants.VisionConstants.MAX_Y_COORDINATE, BlennyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                BlennyConstants.DrivetrainConstants.MAX_LINEAR_SPEED, BlennyConstants.VisionConstants.CAMERA_WIDTH);
        dashboard = new Dashboard(new ReefDisplayIOSwing("ReefDisplay"), new DashboardIOFWC());
        addAutonomousRoutines();
        switch (Constants.getMode())
        {
        case SIM:
        case REAL:
            superstructure = new Superstructure(new Elevator("InnerElevator",
                    new ElevatorIOTalonFX(15, BlennyConstants.InnerElevatorConstants.ELEVATOR_CONSTANTS), new BrakeIO()
                    {
                    }, new ElevatorSensorIOLimitSwitch(1), 0.2),
                    new Elevator("OuterElevator",
                            new ElevatorIOTalonFX(14, BlennyConstants.OuterElevatorConstants.ELEVATOR_CONSTANTS),
                            new BrakeIO()
                            {
                            }, new ElevatorSensorIOLimitSwitch(0), 0.2),
                    new Wrist(new WristIOTalonFX(16, 20, BlennyConstants.WristJointConstants.WRIST_CONSTANTS),
                            Degrees.of(2.0)));
            climber = new Climber(new ClimberIOTalonFX(BlennyConstants.ClimberConstants.ID,
                    BlennyConstants.ClimberConstants.INVERTED, BlennyConstants.ClimberConstants.ENCODER_ID,
                    BlennyConstants.ClimberConstants.LOWER_LIMIT, BlennyConstants.ClimberConstants.UPPER_LIMIT));
            break;
        case REPLAY:
        default:
            superstructure = new Superstructure(new Elevator("InnerElevator", new ElevatorIO()
            {
            }, new BrakeIO()
            {
            }, new ElevatorSensorIO()
            {
            }, 0.2), new Elevator("OuterElevator",
                    new ElevatorIOTalonFX(15, BlennyConstants.OuterElevatorConstants.ELEVATOR_CONSTANTS), new BrakeIO()
                    {
                    }, new ElevatorSensorIOLimitSwitch(1), 0.2), new Wrist(new WristIO()
                    {
                    }, Degrees.of(2.0)));
            climber = new Climber(new ClimberIO()
            {
            });
            break;
        }
        dashboard.setResetEncodersCommand(drive.runOnce(this::resetDriveEncoders).ignoringDisable(true));
        dashboard.setResetWristEncoderCommand(superstructure.getWrist()
                .runOnce(() -> superstructure.getWrist().resetEncoderAngle(Degrees.of(0))).ignoringDisable(true));
    }

    private void addAutonomousRoutines()
    {
        dashboard.addDefaultAutoRoutine("Do Nothing", new AutoRoutine(Commands.none(), new Translation2d[]
        { new Translation2d(), new Translation2d() }, new Pose2d()));
    }

    /**
     * Get the drive subsystem
     * 
     * @return the drive subsystem (PhoenixCommandDrive)
     */
    public PhoenixCommandDrive getDrive()
    {
        return drive;
    }

    /**
     * Get the rollers subsystem from the container
     * 
     * @return the rollers subsystem
     */

    public Rollers getRollers()
    {
        return rollers;
    }

    /**
     * Get the superstructure subsystem from the container
     * 
     * @return the superstructure subsystem
     */
    public Superstructure getSuperstructure()
    {
        return superstructure;
    }

    /**
     * Get the climber subsystem from the container
     * 
     * @return the climber subsystem
     */
    public Climber getClimber()
    {
        return climber;
    }

    /**
     * Get the dashboard
     * 
     * @return the dashboard (Dashboard)
     */
    public Dashboard getDashboard()
    {
        return dashboard;
    }

    @Override
    public void bindDriverOI()
    {
        new BlennyDriverOI().bindOI(this);
    }

    @Override
    public void bindProgrammerOI()
    {
        new BlennyProgrammerOI().bindOI(this);
    }

    @Override
    public Command getAutonomousCommand()
    {
        return dashboard.getRoutine().command();
    }

    @Override
    public void autonomousInit()
    {
        drive.resetPose(
                FieldConstants.convertPoseByAlliance(dashboard.getRoutine().startPose(), FieldConstants.getAlliance()));
    }

    @Override
    public void periodic()
    {
        if (alliance != allianceSupplier.get())
        {
            alliance = allianceSupplier.get();
            drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(allianceSupplier.get()));
        }
        for (var poseEstimate : vision.getPoseEstimates())
        {
            drive.addVisionMeasurement(poseEstimate.pose(), poseEstimate.timestamp());
        }
        dashboard.updatePose(drive.getPose());
        dashboard.setInnerElevatorPosition(superstructure.getInnerElevator().getPosition());
        dashboard.setOuterElevatorPosition(superstructure.getOuterElevator().getPosition());
        hallDisconnectedAlert.set(HALUtil.getHALErrno() == 1);
    }

    public void teleopInit()
    {
        dashboard.setTeleopStage();
    }

    @Override
    public void disabledInit()
    {
        dashboard.setAutoStage();
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

    public boolean isInAlgaeState()
    {
        return inAlgaeState;
    }
}
