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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.ralph.constants.RalphConstants;
import frc.robot.ralph.constants.RalphTunerConstants;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;
import frc.robot.ralph.oi.RalphDriverOI;
import frc.robot.ralph.oi.RalphProgrammerOI;
import frc.robot.subsystems.algaeremover.AlgaeRemover;
import frc.robot.subsystems.algaeremover.AlgaeRemoverIO;
import frc.robot.subsystems.algaeremover.AlgaeRemoverIOTalonFXS;
import frc.robot.subsystems.algaeremover.sensor.AlgaeLimitSwitchIO;
import frc.robot.subsystems.algaeremover.sensor.AlgaeRemoverSensorIO;
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
    private final AlgaeRemover algaeremover;

    /**
     * Create a new RalphContainer
     */
    public RalphContainer()
    {
        drive = new PhoenixCommandDrive(RalphTunerConstants.DrivetrainConstants,
                RalphConstants.DrivetrainConstants.MAX_SPEED, RalphConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                RalphConstants.PathplannerConstants.linearPIDConstants,
                RalphConstants.PathplannerConstants.angularPIDConstants,
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
            algaeremover = new AlgaeRemover(
                    new AlgaeRemoverIOTalonFXS(18, false, RalphConstants.AlgaeRemoverConstants.GEAR_RATIO),
                    new AlgaeLimitSwitchIO(3), RalphConstants.AlgaeRemoverConstants.REMOVING_SPEED,
                    RalphConstants.AlgaeRemoverConstants.RETURNING_SPEED);
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
            algaeremover = new AlgaeRemover(new AlgaeRemoverIO()
            {
            }, new AlgaeRemoverSensorIO()
            {
            }, RalphConstants.AlgaeRemoverConstants.REMOVING_SPEED,
                    RalphConstants.AlgaeRemoverConstants.RETURNING_SPEED);
            break;
        }

        inserter.setDefaultCommand(defaultIntake());
        algaeremover.setDefaultCommand(algaeremover.returnArm());
        dashboard = new Dashboard(new ReefDisplayIOSwing("ReefscapeDisplay"), new DashboardIOFWC());
        RalphAutos.addNamedCommands(this);
        RalphAutos.addAutoRoutines(this);
        dashboard.setResetEncodersCommand(drive.runOnce(this::resetDriveEncoders).ignoringDisable(true));
        PortForwarder.add(5801, "10.1.72.11", 5800);
        PortForwarder.add(5802, "10.1.72.11", 1181);
        PortForwarder.add(5803, "10.1.72.13", 5800);
        PortForwarder.add(5804, "10.1.72.13", 1181);
        PortForwarder.add(5805, "10.1.72.14", 5800);
        PortForwarder.add(5806, "10.1.72.14", 1181);
        PortForwarder.add(5807, "10.1.72.36", 1181);
        PortForwarder.add(5808, "10.1.72.14", 22);
        getInserter().setDefaultCommand(defaultIntake());
    }

    public AlgaeRemover getAlgaeRemover()
    {
        return algaeremover;
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

    public Command getDriveSimple()
    {
        return drive.resetOrientation(Rotation2d.k180deg).andThen(drive.backup(Seconds.of(3), -0.5));
    }

    public Command getDrivePlace()
    {
        return drive.resetOrientation(Rotation2d.k180deg)
                .andThen(drive.backup(Seconds.of(3), -0.5).alongWith(goToL4()).andThen(outtakeCoral()));
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
        return dashboard.getRoutine();
    }

    @Override
    public void autonomousInit()
    {
        if (dashboard.getRoutine().getStartingPose() != null)
            drive.resetPose(dashboard.getRoutine().getStartingPose());
    }

    @Override
    public void periodic()
    {
        vision.updateWithHeading(drive.getHeading());
        if (alliance != allianceSupplier.get())
        {
            alliance = allianceSupplier.get();
            drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(allianceSupplier.get()));
        }
        vision.setLastKnownRobotPose(drive.getPose());
        for (var poseEstimate : vision.getPoseEstimates())
        {
            drive.addVisionMeasurement(poseEstimate.pose(), poseEstimate.timestamp(), VecBuilder.fill(0.1, 0.1, 0.001));
        }
        dashboard.updatePose(drive.getPose());
        dashboard.setInnerElevatorPosition(superstructure.getInnerElevator().getPosition());
        dashboard.setOuterElevatorPosition(superstructure.getOuterElevator().getPosition());
        dashboard.setHasCoral(inserter.hasCoral());
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

    public Distance getDistanceToPose(Pose2d pose)
    {
        return Meters.of(drive.getPose().getTranslation().getDistance(pose.getTranslation()));
    }

    public ReefSide getNearestReefSide()
    {
        ReefSide abSide = FieldConstants.convertReefSideByAlliance(FieldConstants.ReefPositions.AB_SIDE, alliance);
        ReefSide cdSide = FieldConstants.convertReefSideByAlliance(FieldConstants.ReefPositions.CD_SIDE, alliance);
        ReefSide efSide = FieldConstants.convertReefSideByAlliance(FieldConstants.ReefPositions.EF_SIDE, alliance);
        ReefSide ghSide = FieldConstants.convertReefSideByAlliance(FieldConstants.ReefPositions.GH_SIDE, alliance);
        ReefSide ijSide = FieldConstants.convertReefSideByAlliance(FieldConstants.ReefPositions.IJ_SIDE, alliance);
        ReefSide klSide = FieldConstants.convertReefSideByAlliance(FieldConstants.ReefPositions.KL_SIDE, alliance);
        ReefSide nearestSide = abSide;
        Distance nearestDistance = getDistanceToPose(abSide.center());
        if (getDistanceToPose(cdSide.center()).lt(nearestDistance))
        {
            nearestSide = cdSide;
            nearestDistance = getDistanceToPose(cdSide.center());
        }
        if (getDistanceToPose(efSide.center()).lt(nearestDistance))
        {
            nearestSide = efSide;
            nearestDistance = getDistanceToPose(efSide.center());
        }
        if (getDistanceToPose(ghSide.center()).lt(nearestDistance))
        {
            nearestSide = ghSide;
            nearestDistance = getDistanceToPose(ghSide.center());
        }
        if (getDistanceToPose(ijSide.center()).lt(nearestDistance))
        {
            nearestSide = ijSide;
            nearestDistance = getDistanceToPose(ijSide.center());
        }
        if (getDistanceToPose(klSide.center()).lt(nearestDistance))
        {
            nearestSide = klSide;
            nearestDistance = getDistanceToPose(klSide.center());
        }
        return nearestSide;
    }

    public Pose2d applyOffset(Pose2d pose, Distance x, Distance y)
    {
        Translation2d translation = new Translation2d(x, y).rotateBy(pose.getRotation());
        return new Pose2d(pose.getTranslation().plus(translation), pose.getRotation());
    }

    public Pose2d applyOffset(Pose2d pose)
    {
        return applyOffset(pose, Inches.of(1.5), Inches.of(-7));
    }

    public Command driveToLeftReef()
    {
        return Commands.defer(() -> drive.closeDriveToPose(applyOffset(getNearestReefSide().left())), Set.of(drive));
    }

    public Command driveToRightReef()
    {
        return Commands.defer(() -> drive.closeDriveToPose(applyOffset(getNearestReefSide().right())), Set.of(drive));
    }

    public Command driveToCenterReef()
    {
        return Commands.defer(() -> drive.closeDriveToPose(applyOffset(getNearestReefSide().center())), Set.of(drive));
    }

    public Command driveToCenterAlgae()
    {
        return Commands.defer(
                () -> drive.closeDriveToPose(applyOffset(getNearestReefSide().center(), Inches.of(2.5), Inches.of(1))),
                Set.of(drive));
    }

    public Command goToClosestCoralStation()
    {
        if (getDistanceToPose(FieldConstants.convertPoseByAlliance(FieldConstants.CoralStations.LEFT)).in(
                Meters) > getDistanceToPose(FieldConstants.convertPoseByAlliance(FieldConstants.CoralStations.RIGHT))
                        .in(Meters))
        {
            return drive.closeDriveToPose(FieldConstants.CoralStations.RIGHT);
        } else
        {
            return drive.closeDriveToPose(FieldConstants.CoralStations.LEFT);
        }
    }

    public Command goToProcessor()
    {
        return drive.closeDriveToPose(FieldConstants.ProcessorStations.PROCESSOR_STATION);
    }

    public Command getBackupAuto()
    {
        return drive.backup(Seconds.of(1), 0.5);
    }
}
