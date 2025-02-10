package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.reefscape.ReefDisplayIO;
import frc.robot.subsystems.reefscape.ReefDisplayIOInputsAutoLogged;
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

    public void addAutoRoutine(String name, AutoRoutine command)
    {
        m_io.addRoutine(name, command, false);
    }

    public void addDefaultAutoRoutine(String name, AutoRoutine command)
    {
        m_io.addRoutine(name, command, true);
    }

    public void setAutoStage()
    {
        m_io.setStage(DashboardIO.DashboardIOStage.AUTO);
    }

    public void setTeleopStage()
    {
        m_io.setStage(DashboardIO.DashboardIOStage.TELEOP);
    }

    public void setSettingsStage()
    {
        m_io.setStage(DashboardIO.DashboardIOStage.SETTINGS);
    }

    public void setResetEncodersCommand(Command command)
    {
        m_io.addCommand("ResetEncoders", command);
    }

    @Override
    public void periodic()
    {
        m_io.updateInputs(m_input);
        Logger.processInputs(getName() + "/Dashboard", m_input);
        reefDisplayIO.updateInputs(reefDisplayInputs);
        Logger.processInputs(getName() + "/ReefDisplayIO", reefDisplayInputs);
    }

    public AutoRoutine getRoutine()
    {
        return m_io.getSelectedRoutine();
    }
}