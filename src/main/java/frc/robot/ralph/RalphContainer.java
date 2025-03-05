package frc.robot.ralph;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import org.northernforce.util.NFRRobotContainer;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefPositions.ReefSide;
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
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.subsystems.photonvision.PhotonVision;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.elevator.brake.BrakeIO;
import frc.robot.subsystems.superstructure.elevator.sensor.ElevatorSensorIO;
import frc.robot.subsystems.superstructure.elevator.sensor.ElevatorSensorIOLimitSwitch;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOTalonFXS;
import frc.robot.subsystems.rollers.sensor.RollersSensorIO;
import frc.robot.subsystems.rollers.sensor.RollersSensorIOAnalog;
import frc.robot.subsystems.rollers.sensor.RollersSensorIOBeamBreak;
import frc.robot.subsystems.viewer.Viewer;
import frc.robot.subsystems.viewer.ViewerIO;
import frc.robot.subsystems.viewer.ViewerIOXavier;

/**
 * 2025 Competition Robot Container.
 */
public class RalphContainer implements NFRRobotContainer
{
    private final PhoenixCommandDrive drive;
    private final Rollers rollers;
    private final Superstructure superstructure;
    private final PhotonVision vision;
    private final Supplier<Alliance> allianceSupplier = () -> DriverStation.getAlliance().orElse(Alliance.Red);
    private Alliance alliance = allianceSupplier.get();
    private final Climber climber;
    private final Dashboard dashboard;
    private final Viewer viewer;

    private final Supplier<Boolean> useBeamBreak = () -> SmartDashboard.getBoolean("Use Beam Break", true);

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
            rollers = new Rollers(
                    new RollersIOTalonFXS(RalphConstants.RollersConstants.ROLLER_MOTOR_LEFT_ID,
                            RalphConstants.RollersConstants.ROLLER_MOTOR_RIGHT_ID,
                            RalphConstants.RollersConstants.ROLLER_MOTORS_INVERTED),
                    new RollersSensorIOAnalog(RalphConstants.RollersConstants.SensorConstants.ANALOG_ALGAE,
                            RalphConstants.RollersConstants.SensorConstants.ALGAE_MAX_DISTANCE),
                    new RollersSensorIOBeamBreak(RalphConstants.RollersConstants.SensorConstants.ANALOG_CORAL),
                    RalphConstants.RollersConstants.INTAKE_SPEED, RalphConstants.RollersConstants.OUTTAKE_SPEED);
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
            rollers = new Rollers(new RollersIO()
            {
            }, new RollersSensorIO()
            {
            }, new RollersSensorIO()
            {
            }, RalphConstants.RollersConstants.INTAKE_SPEED, RalphConstants.RollersConstants.OUTTAKE_SPEED);
            break;
        }
        dashboard = new Dashboard(new ReefDisplayIOSwing("ReefscapeDisplay"), new DashboardIOFWC());
        RalphAutos.addAutoRoutines(this);
        dashboard.setResetEncodersCommand(drive.runOnce(this::resetDriveEncoders).ignoringDisable(true));
        SmartDashboard.putData("Go do thing to the left", getGoToReefPoseCommandLeft());
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
        double angleRadians = closestTranslation.center().getRotation().getRadians() + 3 * Math.PI / 2;
        double xOffset = Inches.of(Math.cos(angleRadians) * -10).in(Meters);
        double yOffset = Inches.of(Math.sin(angleRadians) * -10).in(Meters);
        Pose2d finalTranslatedPose2d = new Pose2d(closestTranslation.center().getTranslation().getX() + xOffset,
                closestTranslation.center().getTranslation().getY() + yOffset,
                closestTranslation.center().getRotation());
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
        return Commands.defer(
                () -> getDrive()
                        .driveToPose((getTargetPoseLeft(getDrive().getPose())),
                                RalphConstants.PathplannerConstants.MAX_VELOCITY,
                                RalphConstants.PathplannerConstants.MAX_ACCELERATION,
                                RalphConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                                RalphConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION)
                        .alongWith(
                                getSuperstructure().getGoToGoalCommand(getDashboard().getSuperstructureGoalForReef())),
                Set.of());
    }

    public Command getGoToReefPoseCommandRight()
    {
        return Commands.defer(
                () -> getDrive()
                        .driveToPose((getTargetPoseRight(getDrive().getPose())),
                                RalphConstants.PathplannerConstants.MAX_VELOCITY,
                                RalphConstants.PathplannerConstants.MAX_ACCELERATION,
                                RalphConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                                RalphConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION)
                        .alongWith(
                                getSuperstructure().getGoToGoalCommand(getDashboard().getSuperstructureGoalForReef())),
                Set.of());
    }

    public Command getGoToReefPoseCommandCenter()
    {
        return Commands.defer(
                () -> getDrive()
                        .driveToPose((getTargetPoseCenter(getDrive().getPose())),
                                RalphConstants.PathplannerConstants.MAX_VELOCITY,
                                RalphConstants.PathplannerConstants.MAX_ACCELERATION,
                                RalphConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                                RalphConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION)
                        .alongWith(
                                getSuperstructure().getGoToGoalCommand(getDashboard().getSuperstructureGoalForReef())),
                Set.of());
    }

    public Command getGoToStationCommand()
    {
        return Commands.defer(() -> getDrive()
                .driveToPose(getDashboard().getStationTargetPose(), RalphConstants.PathplannerConstants.MAX_VELOCITY,
                        RalphConstants.PathplannerConstants.MAX_ACCELERATION,
                        RalphConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                        RalphConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION)
                .alongWith(getSuperstructure().getGoToGoalCommand(getDashboard().getSuperstructureGoalForStation())),
                Set.of());
    }

    public Command getCoralIntakeCommand()
    {
        return // superstructure.getGoToGoalCommand(SuperstructureGoal.CORAL_STATION)
               // .andThen(rollers.getCoralIntakeCommand());
        rollers.getCoralIntakeCommand(useBeamBreak.get());
    }

    public Command getCoralOuttakeCommand()
    {
        return rollers.getOuttakeCoralCommand().andThen(drive.backup(Seconds.of(3), 0.3));
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
        vision.setLastKnownRobotPose(drive.getPose());
        for (var poseEstimate : vision.getPoseEstimates())
        {
            drive.addVisionMeasurement(poseEstimate.pose(), Utils.fpgaToCurrentTime(poseEstimate.timestamp()));
        }
        dashboard.updatePose(drive.getPose());
        dashboard.setInnerElevatorPosition(superstructure.getInnerElevator().getPosition());
        dashboard.setOuterElevatorPosition(superstructure.getOuterElevator().getPosition());
        dashboard.setHasCoral(rollers.hasCoral());
        dashboard.setHasAlgae(rollers.hasAlgae());
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
        return superstructure.getGoal() == SuperstructureGoal.HIGHER_ALGAE
                || superstructure.getGoal() == SuperstructureGoal.LOWER_ALGAE
                || superstructure.getGoal() == SuperstructureGoal.PROCESSOR_STATION
                || superstructure.getGoal() == SuperstructureGoal.STOW_ALGAE;
    }

    public Command getOuttakeCommand()
    {
        return isInAlgaeState() ? rollers.getOuttakeCommand() : getCoralOuttakeCommand();
    }

    public Command getOuttakeCommand(double speed)
    {
        return isInAlgaeState() ? rollers.getOuttakeCommand(speed) : getCoralOuttakeCommand();
    }
}
