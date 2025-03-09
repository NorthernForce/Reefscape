package frc.robot.ralph;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import org.northernforce.util.NFRRobotContainer;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefPositions.ReefSide;
import frc.robot.ralph.constants.RalphConstants;
import frc.robot.ralph.constants.RalphTunerConstants;
import frc.robot.ralph.constants.RalphConstants.DrivetrainConstants;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;
import frc.robot.ralph.oi.RalphDriverOI;
import frc.robot.ralph.oi.RalphOI;
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
    private Translation2d drivePose;
    private Rotation2d driveAngle;

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
                RalphConstants.AutoConstants.yPID, RalphConstants.AutoConstants.rPID, RalphConstants.AutoConstants.kP,
                RalphConstants.AutoConstants.kI, RalphConstants.AutoConstants.kD, RalphConstants.AutoConstants.postP,
                RalphConstants.AutoConstants.postI, RalphConstants.AutoConstants.postD,
                RalphConstants.AutoConstants.kConstraints, RalphConstants.AutoConstants.kPRotation,
                RalphConstants.AutoConstants.rotationContinuous, RalphConstants.AutoConstants.totalAngle,
                RalphConstants.AutoConstants.totalDistance, RalphConstants.DrivetrainConstants.SWERVE_MODULE_OFFSETS,
                RalphTunerConstants.FrontLeft, RalphTunerConstants.FrontRight, RalphTunerConstants.BackLeft,
                RalphTunerConstants.BackRight);
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
    }

    public Pose2d getTargetPoseLeft()
    {
        ReefSide closestTranslation = FieldConstants.ReefPositions.SIDES[0];
        for (ReefSide pose : FieldConstants.ReefPositions.SIDES)
        {
            if (pose.getDistanceFromLeft(drivePose).in(Meters) < closestTranslation.getDistanceFromCenter(drivePose)
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

    public Pose2d getTargetPoseCenter()
    {
        Pose2d currentPose = new Pose2d(getDrive().getPose().getTranslation(), getDrive().getPose().getRotation());
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

    public Pose2d getTargetPoseRight()
    {
        Pose2d currentPose = new Pose2d(drive.getPose().getTranslation(), drive.getPose().getRotation());

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
        return Commands.defer(() -> getDrive().driveToAccurate(getTargetPoseLeft(),
                DrivetrainConstants.MAX_LINEAR_SPEED, DrivetrainConstants.MAX_ACCELERATION,
                DrivetrainConstants.MAX_ANGULAR_SPEED, DrivetrainConstants.MAX_ANGULAR_ACCELERATION,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KP, DrivetrainConstants.CLOSE_TRANSLATION_PP_KI,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KD, DrivetrainConstants.CLOSE_ROTATION_PP_KP,
                DrivetrainConstants.CLOSE_ROTATION_PP_KI, DrivetrainConstants.CLOSE_ROTATION_PP_KD), Set.of());
    }

    public Command getGoToReefPoseCommandRight()
    {
        return Commands.defer(() -> getDrive().driveToAccurate(getTargetPoseRight(),
                DrivetrainConstants.MAX_LINEAR_SPEED, DrivetrainConstants.MAX_ACCELERATION,
                DrivetrainConstants.MAX_ANGULAR_SPEED, DrivetrainConstants.MAX_ANGULAR_ACCELERATION,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KP, DrivetrainConstants.CLOSE_TRANSLATION_PP_KI,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KD, DrivetrainConstants.CLOSE_ROTATION_PP_KP,
                DrivetrainConstants.CLOSE_ROTATION_PP_KI, DrivetrainConstants.CLOSE_ROTATION_PP_KD), Set.of());
    }

    public Command getGoToReefPoseCommandCenter()
    {
        return Commands.defer(() -> getDrive().driveToAccurate(getTargetPoseCenter(),
                DrivetrainConstants.MAX_LINEAR_SPEED, DrivetrainConstants.MAX_ACCELERATION,
                DrivetrainConstants.MAX_ANGULAR_SPEED, DrivetrainConstants.MAX_ANGULAR_ACCELERATION,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KP, DrivetrainConstants.CLOSE_TRANSLATION_PP_KI,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KD, DrivetrainConstants.CLOSE_ROTATION_PP_KP,
                DrivetrainConstants.CLOSE_ROTATION_PP_KI, DrivetrainConstants.CLOSE_ROTATION_PP_KD), Set.of());
    }

    public Pose2d getClosestCoralStation()
    {
        Pose2d pose = new Pose2d(getDrive().getPose().getTranslation(), getDrive().getPose().getRotation());
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
        return Commands.defer(() -> getDrive().driveToAccurate(getClosestCoralStation(),
                DrivetrainConstants.MAX_LINEAR_SPEED, DrivetrainConstants.MAX_ACCELERATION,
                DrivetrainConstants.MAX_ANGULAR_SPEED, DrivetrainConstants.MAX_ANGULAR_ACCELERATION,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KP, DrivetrainConstants.CLOSE_TRANSLATION_PP_KI,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KD, DrivetrainConstants.CLOSE_ROTATION_PP_KP,
                DrivetrainConstants.CLOSE_ROTATION_PP_KI, DrivetrainConstants.CLOSE_ROTATION_PP_KD), Set.of());
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
        drivePose = drive.getPose().getTranslation();
        driveAngle = drive.getPose().getRotation();
        if (alliance != allianceSupplier.get())
        {
            alliance = allianceSupplier.get();
            drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(allianceSupplier.get()));
        }
        vision.setLastKnownRobotPose(new Pose2d(drivePose, driveAngle));
        for (var poseEstimate : vision.getPoseEstimates())
        {
            drive.addVisionMeasurement(poseEstimate.pose(), Utils.fpgaToCurrentTime(poseEstimate.timestamp()));
        }
        dashboard.updatePose(new Pose2d(drivePose, driveAngle));
        dashboard.setInnerElevatorPosition(superstructure.getInnerElevator().getPosition());
        dashboard.setOuterElevatorPosition(superstructure.getOuterElevator().getPosition());
        dashboard.setHasCoral(inserter.hasCoral());
    }

    public Pose2d getDrivePose()
    {
        return new Pose2d(drivePose, driveAngle);
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

    public Command driveToPose(Pose2d pose)
    {
        return Commands.defer(() -> getDrive().driveToAccurate(pose, DrivetrainConstants.MAX_LINEAR_SPEED,
                DrivetrainConstants.MAX_ACCELERATION, DrivetrainConstants.MAX_ANGULAR_SPEED,
                DrivetrainConstants.MAX_ANGULAR_ACCELERATION, DrivetrainConstants.CLOSE_TRANSLATION_PP_KP,
                DrivetrainConstants.CLOSE_TRANSLATION_PP_KI, DrivetrainConstants.CLOSE_TRANSLATION_PP_KD,
                DrivetrainConstants.CLOSE_ROTATION_PP_KP, DrivetrainConstants.CLOSE_ROTATION_PP_KI,
                DrivetrainConstants.CLOSE_ROTATION_PP_KD), Set.of());

    }
}
