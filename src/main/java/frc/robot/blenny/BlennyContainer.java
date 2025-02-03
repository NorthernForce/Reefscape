package frc.robot.blenny;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.blenny.constants.BlennyConstants;
import frc.robot.blenny.constants.BlennyTunerConstants;
import frc.robot.blenny.oi.BlennyDriverOI;
import frc.robot.blenny.oi.BlennyProgrammerOI;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.dashboard.DashboardIOFWC;
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.subsystems.reefscape.ReefDisplayIOSwing;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.elevator.brake.BrakeIO;
import frc.robot.subsystems.superstructure.elevator.brake.BrakeIORelay;
import frc.robot.subsystems.superstructure.elevator.sensor.ElevatorSensorIO;
import frc.robot.subsystems.superstructure.elevator.sensor.ElevatorSensorIOLimitSwitch;
import frc.robot.util.AutoRoutine;

/**
 * 2025 Competition Robot Container. Name is still a work in progress and will
 * likely change. Blenny is a type of fish. It is also the name of a submarine
 * that was sunk to create an artificial reef.
 */
public class BlennyContainer implements NFRRobotContainer
{
    private final PhoenixCommandDrive drive;
    private final Superstructure superstructure;
    private final Dashboard dashboard;

    /**
     * Create a new BlennyContainer
     */
    public BlennyContainer()
    {
        drive = new PhoenixCommandDrive(BlennyTunerConstants.DrivetrainConstants,
                BlennyConstants.DrivetrainConstants.MAX_SPEED, BlennyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                BlennyTunerConstants.FrontLeft, BlennyTunerConstants.FrontRight, BlennyTunerConstants.BackLeft,
                BlennyTunerConstants.BackRight);
        dashboard = new Dashboard(new ReefDisplayIOSwing("ReefscapeDisplay"), new DashboardIOFWC());
        addAutonomousRoutines();
        switch (Constants.kCurrentMode)
        {
        case SIM:
        case REAL:
            superstructure = new Superstructure(
                    new Elevator("InnerElevator",
                            new ElevatorIOTalonFX(14, BlennyConstants.InnerElevatorConstants.ELEVATOR_CONSTANTS),
                            new BrakeIORelay(0), new ElevatorSensorIOLimitSwitch(0), 0.2),
                    new Elevator("OuterElevator",
                            new ElevatorIOTalonFX(15, BlennyConstants.OuterElevatorConstants.ELEVATOR_CONSTANTS),
                            new BrakeIORelay(1), new ElevatorSensorIOLimitSwitch(1), 0.2));
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
                    new ElevatorIOTalonFX(15, BlennyConstants.OuterElevatorConstants.ELEVATOR_CONSTANTS),
                    new BrakeIORelay(1), new ElevatorSensorIOLimitSwitch(1), 0.2));
            break;
        }
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
     * Get the superstructure subsystem
     * 
     * @return the superstructure subsystem (Superstructure)
     */
    public Superstructure getSuperstructure()
    {
        return superstructure;
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
    public void bindOI()
    {
        switch (Constants.kOI)
        {
        case PROGRAMMER:
            new BlennyProgrammerOI().bindOI(this);
            break;
        case DRIVER:
        default:
            new BlennyDriverOI().bindOI(this);
            break;
        }
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

}
