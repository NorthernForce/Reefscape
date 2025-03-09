package frc.robot.ralph;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import org.northernforce.util.NFRRobotContainer;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.ralph.constants.RalphConstants;
import frc.robot.ralph.constants.RalphTunerConstants;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;
import frc.robot.ralph.oi.RalphDriverOI;
import frc.robot.ralph.oi.RalphProgrammerOI;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.dashboard.DashboardIOFWC;
import frc.robot.subsystems.dashboard.reefscape.ReefDisplayIOSwing;
import frc.robot.subsystems.inserter.Inserter;
import frc.robot.subsystems.inserter.InserterIO;
import frc.robot.subsystems.inserter.InserterIOTalonFXS;
import frc.robot.subsystems.inserter.sensor.InserterSensorIO;
import frc.robot.subsystems.inserter.sensor.InserterSensorIOBeamBreak;
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.subsystems.photonvision.PhotonVision;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.elevator.brake.BrakeIO;
import frc.robot.subsystems.superstructure.elevator.sensor.ElevatorSensorIO;
import frc.robot.subsystems.superstructure.elevator.sensor.ElevatorSensorIOLimitSwitch;
import frc.robot.subsystems.viewer.Viewer;
import frc.robot.subsystems.viewer.ViewerIO;
import frc.robot.subsystems.viewer.ViewerIOXavier;

/**
 * 2025 Competition Robot Container.
 */
public class RalphContainer implements NFRRobotContainer
{
    private final PhoenixCommandDrive drive;
    private final Inserter inserter;
    private final Superstructure superstructure;
    private final PhotonVision vision;
    private final Supplier<Alliance> allianceSupplier = () -> DriverStation.getAlliance().orElse(Alliance.Red);
    private Alliance alliance = allianceSupplier.get();
    private final Climber climber;
    private final Dashboard dashboard;
    private final Viewer viewer;

    /**
     * Create a new RalphContainer
     */
    public RalphContainer()
    {

        drive = new PhoenixCommandDrive(RalphTunerConstants.DrivetrainConstants,
                RalphConstants.DrivetrainConstants.MAX_SPEED, RalphConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                RalphConstants.PathplannerConstants.linearPIDConstants,
                RalphConstants.PathplannerConstants.angularPIDConstants,
                RalphConstants.DrivetrainConstants.SAFE_DISTANCE, RalphConstants.AutoConstants.xPID,
                RalphConstants.AutoConstants.yPID, RalphConstants.AutoConstants.rPID,
                RalphConstants.DrivetrainConstants.SWERVE_MODULE_OFFSETS, RalphTunerConstants.FrontLeft,
                RalphTunerConstants.FrontRight, RalphTunerConstants.BackLeft, RalphTunerConstants.BackRight);
        drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(allianceSupplier.get()));

        vision = new PhotonVision(RalphConstants.VisionConstants.cameraNames(),
                RalphConstants.VisionConstants.cameraTransforms(), RalphConstants.VisionConstants.APRILTAG_LAYOUT,
                RalphConstants.VisionConstants.MAX_Y_COORDINATE, RalphConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                RalphConstants.DrivetrainConstants.MAX_LINEAR_SPEED, RalphConstants.VisionConstants.CAMERA_WIDTH);
        switch (Constants.getMode())
        {
        case SIM:
        case REAL:
            superstructure = new Superstructure(new Elevator("InnerElevator",
                    new ElevatorIOTalonFX(15, RalphConstants.InnerElevatorConstants.ELEVATOR_CONSTANTS), new BrakeIO()
                    {
                    }, new ElevatorSensorIOLimitSwitch(0), Inches.of(0.5)),
                    new Elevator("OuterElevator",
                            new ElevatorIOTalonFX(14, RalphConstants.OuterElevatorConstants.ELEVATOR_CONSTANTS),
                            new BrakeIO()
                            {
                            }, new ElevatorSensorIOLimitSwitch(1), Inches.of(0.5)),
                    RalphConstants.InnerElevatorConstants.HIGH_POSITION,
                    RalphConstants.OuterElevatorConstants.HIGH_POSITION);
            climber = new Climber(
                    new ClimberIOTalonFX(RalphConstants.ClimberConstants.ID, RalphConstants.ClimberConstants.INVERTED,
                            RalphConstants.ClimberConstants.ENCODER_ID, RalphConstants.ClimberConstants.LOWER_LIMIT,
                            RalphConstants.ClimberConstants.UPPER_LIMIT),
                    RalphConstants.ClimberConstants.SWEET_ANGLE, RalphConstants.ClimberConstants.LOWER_LIMIT,
                    RalphConstants.ClimberConstants.UPPER_LIMIT, RalphConstants.ClimberConstants.CLIMB_SPEED);

            viewer = new Viewer(new ViewerIOXavier());
            inserter = new Inserter(
                    new InserterIOTalonFXS(RalphConstants.InserterConstants.ROLLER_MOTOR_ID,
                            RalphConstants.InserterConstants.ROLLER_MOTOR_INVERTED),
                    new InserterSensorIOBeamBreak(RalphConstants.InserterConstants.SensorConstants.CORAL_PIN),
                    RalphConstants.InserterConstants.INTAKE_SPEED, RalphConstants.InserterConstants.OUTTAKE_SPEED);
            break;
        case REPLAY:
        default:
            superstructure = new Superstructure(new Elevator("InnerElevator", new ElevatorIO()
            {
            }, new BrakeIO()
            {
            }, new ElevatorSensorIO()
            {
            }, Inches.of(0.5)), new Elevator("OuterElevator",
                    new ElevatorIOTalonFX(15, RalphConstants.OuterElevatorConstants.ELEVATOR_CONSTANTS), new BrakeIO()
                    {
                    }, new ElevatorSensorIOLimitSwitch(1), Inches.of(0.5)),
                    RalphConstants.InnerElevatorConstants.HIGH_POSITION,
                    RalphConstants.OuterElevatorConstants.HIGH_POSITION);
            climber = new Climber(new ClimberIO()
            {
            }, RalphConstants.ClimberConstants.SWEET_ANGLE, RalphConstants.ClimberConstants.LOWER_LIMIT,
                    RalphConstants.ClimberConstants.UPPER_LIMIT, RalphConstants.ClimberConstants.CLIMB_SPEED);
            viewer = new Viewer(new ViewerIO()
            {
            });
            inserter = new Inserter(new InserterIO()
            {
            }, new InserterSensorIO()
            {
            }, RalphConstants.InserterConstants.INTAKE_SPEED, RalphConstants.InserterConstants.OUTTAKE_SPEED);
            break;
        }
        dashboard = new Dashboard(new ReefDisplayIOSwing("ReefscapeDisplay"), new DashboardIOFWC());
        RalphAutos.addAutoRoutines(this);
        dashboard.setResetEncodersCommand(drive.runOnce(this::resetDriveEncoders).ignoringDisable(true));
        SmartDashboard.putData("Go do thing", getGoToReefPoseCommand());
    }

    public Command getGoToReefPoseCommand()
    {
        return Commands.defer(() -> getDrive()
                .driveToPose(FieldConstants.getReefBackupPosition(getDashboard().getTargetPose(), Feet.of(1)),
                        RalphConstants.PathplannerConstants.MAX_VELOCITY,
                        RalphConstants.PathplannerConstants.MAX_ACCELERATION,
                        RalphConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                        RalphConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION)
                .alongWith(getSuperstructure().goToGoal(getDashboard().getSuperstructureGoalForReef()))
                .andThen(() -> getDrive().driveToPose(getDashboard().getTargetPose(),
                        RalphConstants.PathplannerConstants.MAX_VELOCITY,
                        RalphConstants.PathplannerConstants.MAX_ACCELERATION,
                        RalphConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                        RalphConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION)),
                Set.of());
    }

    public Command getGoToStationCommand()
    {
        return Commands.defer(() -> getDrive()
                .driveToPose(getDashboard().getStationTargetPose(), RalphConstants.PathplannerConstants.MAX_VELOCITY,
                        RalphConstants.PathplannerConstants.MAX_ACCELERATION,
                        RalphConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                        RalphConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION)
                .alongWith(getSuperstructure().goToGoal(getDashboard().getSuperstructureGoalForStation())), Set.of());
    }

    public Command intakeCoral()
    {
        return inserter.intakeCoral();
    }

    public Command outtakeCoral()
    {
        return inserter.outtakeCoral();
    }

    public Command goToL4()
    {
        return superstructure.goToGoal(SuperstructureGoal.L4);
    }

    public Command goToL3()
    {
        return superstructure.goToGoal(SuperstructureGoal.L3);
    }

    public Command goToL2()
    {
        return superstructure.goToGoal(SuperstructureGoal.L2);
    }

    public Command goToL1()
    {
        return superstructure.goToGoal(SuperstructureGoal.L1);
    }

    public Command holdAtL4()
    {
        return superstructure.holdAtGoal(SuperstructureGoal.L4);
    }

    public Command holdAtL3()
    {
        return superstructure.holdAtGoal(SuperstructureGoal.L3);
    }

    public Command holdAtL2()
    {
        return superstructure.holdAtGoal(SuperstructureGoal.L2);
    }

    public Command holdAtL1()
    {
        return superstructure.holdAtGoal(SuperstructureGoal.L1);
    }

    public Command goToIntake()
    {
        return superstructure.goToGoal(SuperstructureGoal.CORAL_STATION);
    }

    public Command homeElevator()
    {
        return superstructure.getHomingCommand(RalphConstants.InnerElevatorConstants.HOMING_SPEED,
                RalphConstants.OuterElevatorConstants.HOMING_SPEED);
    }

    public Command driveByJoystick(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier)
    {
        return drive.driveByJoystick(xSupplier, ySupplier, rSupplier);
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
     * Get the inserter subsystem from the container
     * 
     * @return the inserter subsystem
     */

    public Inserter getInserter()
    {
        return inserter;
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

    public Viewer getViewer()
    {
        return viewer;
    }

    @Override
    public void bindDriverOI()
    {
        new RalphDriverOI().bindOI(this);
    }

    @Override
    public void bindProgrammerOI()
    {
        new RalphProgrammerOI().bindOI(this);
    }

    @Override
    public Command getAutonomousCommand()
    {
        return dashboard.getRoutine().command();
    }

    @Override
    public void autonomousInit()
    {
        drive.resetPose(dashboard.getRoutine().startPose().get());
    }

    @Override
    public void periodic()
    {
        if (alliance != allianceSupplier.get())
        {
            alliance = allianceSupplier.get();
            drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(allianceSupplier.get()));
        }
        vision.setReferencePose(drive.getPose());
        for (var poseEstimate : vision.getPoseEstimates())
        {
            drive.addVisionMeasurement(poseEstimate.pose(), Utils.fpgaToCurrentTime(poseEstimate.timestamp()));
        }
        dashboard.updatePose(drive.getPose());
        dashboard.setInnerElevatorPosition(superstructure.getInnerElevator().getPosition());
        dashboard.setOuterElevatorPosition(superstructure.getOuterElevator().getPosition());
        dashboard.setHasCoral(inserter.hasCoral());
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

    public class IntakeWhileWaitingCommand extends Command
    {
        public IntakeWhileWaitingCommand()
        {
            addRequirements(inserter);
        }

        @Override
        public void execute()
        {
            if (!inserter.hasCoral() && superstructure.isAtGoal(SuperstructureGoal.CORAL_STATION))
            {
                inserter.intake();
            } else
            {
                inserter.stop();
            }
        }
    }

    public Command defaultIntake()
    {
        return new IntakeWhileWaitingCommand();
    }
}
