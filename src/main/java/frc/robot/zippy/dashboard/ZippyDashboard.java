package frc.robot.zippy.dashboard;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.reefscape.ReefDisplayIO;
import frc.robot.subsystems.reefscape.ReefDisplayIOInputsAutoLogged;

/**
 * Subsystem for the Zippy dashboard.
 */
public class ZippyDashboard extends SubsystemBase
{
    private final ZippyDashboardIO m_io;
    private final ZippyDashboardIOInputsAutoLogged m_input;
    private final ReefDisplayIO reefDisplayIO;
    private final ReefDisplayIOInputsAutoLogged reefDisplayInputs;

    /**
     * Constructs a new ZippyDashboard.
     */
    public ZippyDashboard(ReefDisplayIO displayIO, ZippyDashboardIO io)
    {
        this.m_io = io;
        this.m_input = new ZippyDashboardIOInputsAutoLogged();
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

    public void addAutoCommand(String name, Command command)
    {
        m_io.addCommand(name, command, false);
    }

    public void addDefaultAutoCommand(String name, Command command)
    {
        m_io.addCommand(name, command, true);
    }

    public void setAutoStage()
    {
        m_io.setStage(ZippyDashboardIO.ZippyDashboardIOStage.AUTO);
    }

    public void setTeleopStage()
    {
        m_io.setStage(ZippyDashboardIO.ZippyDashboardIOStage.TELEOP);
    }

    public void setSettingsStage()
    {
        m_io.setStage(ZippyDashboardIO.ZippyDashboardIOStage.SETTINGS);
    }

    @Override
    public void periodic()
    {
        m_io.updateInputs(m_input);
        Logger.processInputs(getName() + "/Dashboard", m_input);
        reefDisplayIO.updateInputs(reefDisplayInputs);
        Logger.processInputs(getName() + "/ReefDisplayIO", reefDisplayInputs);
    }
}