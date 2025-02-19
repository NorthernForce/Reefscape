package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.dashboard.reefscape.ReefDisplayIO;
import frc.robot.subsystems.dashboard.reefscape.ReefDisplayIOInputsAutoLogged;
import frc.robot.util.AutoRoutine;

/**
 * Subsystem for the dashboard.
 */
public class Dashboard extends SubsystemBase
{
    private final DashboardIO m_io;
    private final DashboardIOInputsAutoLogged m_input;
    private final ReefDisplayIO reefDisplayIO;
    private final ReefDisplayIOInputsAutoLogged reefDisplayInputs;

    /**
     * Constructs a new Dashboard.
     */
    public Dashboard(ReefDisplayIO displayIO, DashboardIO io)
    {
        this.m_io = io;
        this.m_input = new DashboardIOInputsAutoLogged();
        this.reefDisplayIO = displayIO;
        this.reefDisplayInputs = new ReefDisplayIOInputsAutoLogged();
    }

    /**
     * Gets the target pose for the reef location.
     * 
     * @return The target pose.
     */
    @AutoLogOutput
    public Pose2d getTargetPose()
    {
        return FieldConstants.REEF_POSITIONS.get(reefDisplayInputs.reefLocations);
    }

    /**
     * Adds an auto routine to the dashboard.
     * 
     * @param name    Auto routine name (Descriptive for drivers please)
     * @param command Auto routine command
     */
    public void addAutoRoutine(String name, AutoRoutine command)
    {
        m_io.addRoutine(name, command, false);
    }

    /**
     * Adds a default auto routine to the dashboard.
     * 
     * @param name    Auto routine name (Descriptive for drivers please)
     * @param command Auto routine command
     */
    public void addDefaultAutoRoutine(String name, AutoRoutine command)
    {
        m_io.addRoutine(name, command, true);
    }

    /**
     * Sets the stage of the dashboard to auto. This only changes the display stage
     * when toggle is on.
     */
    public void setAutoStage()
    {
        m_io.setStage(DashboardIO.DashboardIOStage.AUTO);
    }

    /**
     * Sets the stage of the dashboard to teleop. This only changes the display
     * stage when toggle is on.
     */
    public void setTeleopStage()
    {
        m_io.setStage(DashboardIO.DashboardIOStage.TELEOP);
    }

    /**
     * Sets the stage of the dashboard to settings. This only changes the display
     * stage when toggle is on.
     */
    public void setSettingsStage()
    {
        m_io.setStage(DashboardIO.DashboardIOStage.SETTINGS);
    }

    public void setResetEncodersCommand(Command command)
    {
        m_io.addCommand("ResetSwerveWheels", command);
    }

    public void setResetWristEncoderCommand(Command command)
    {
        m_io.addCommand("ResetWrist", command);
    }

    public void setInnerElevatorGoToPosition(Command command)
    {
        m_io.addCommand("InnerElevator/GoToPosition", command);
    }

    public void setOuterElevatorGoToPosition(Command command)
    {
        m_io.addCommand("OuterElevator/GoToPosition", command);
    }

    public void setInnerElevatorPosition(Distance position)
    {
        m_io.setInnerElevatorPosition(position);
    }

    public void setOuterElevatorPosition(Distance position)
    {
        m_io.setOuterElevatorPosition(position);
    }

    public Distance getInnerElevatorTargetPosition()
    {
        return m_input.innerElevatorTargetPosition;
    }

    public Distance getOuterElevatorTargetPosition()
    {
        return m_input.outerElevatorTargetPosition;
    }

    public void setResetWristEncoderCommand(Command command)
    {
        m_io.addCommand("Wrist/ResetEncoder", command);
    }

    public void setInnerElevatorGoToPosition(Command command)
    {
        m_io.addCommand("InnerElevator/GoToPosition", command);
    }

    public void setOuterElevatorGoToPosition(Command command)
    {
        m_io.addCommand("OuterElevator/GoToPosition", command);
    }

    public void setInnerElevatorPosition(Distance position)
    {
        m_io.setInnerElevatorPosition(position);
    }

    public void setOuterElevatorPosition(Distance position)
    {
        m_io.setOuterElevatorPosition(position);
    }

    public Distance getInnerElevatorTargetPosition()
    {
        return m_input.innerElevatorTargetPosition;
    }

    public Distance getOuterElevatorTargetPosition()
    {
        return m_input.outerElevatorTargetPosition;
    }

    public void updatePose(Pose2d pose)
    {
        m_io.updatePose(pose);
    }

    @Override
    public void periodic()
    {
        m_io.updateInputs(m_input);
        Logger.processInputs(getName() + "/Dashboard", m_input);
        reefDisplayIO.updateInputs(reefDisplayInputs);
        Logger.processInputs(getName() + "/ReefDisplayIO", reefDisplayInputs);
        m_io.setTime(DriverStation.getMatchTime());
    }

    /**
     * Gets the selected auto routine. It will return the default auto routine if no
     * routine is selected.
     * 
     * @return The selected auto routine.
     */
    public AutoRoutine getRoutine()
    {
        return m_io.getSelectedRoutine();
    }
}