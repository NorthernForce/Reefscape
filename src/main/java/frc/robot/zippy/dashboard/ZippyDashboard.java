package frc.robot.zippy.dashboard;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.reefscape.ReefDisplayIO;
import frc.robot.subsystems.reefscape.ReefDisplayIOInputsAutoLogged;

/**
 * Subsystem for the Zippy dashboard.
 */
public class ZippyDashboard extends SubsystemBase
{
	private final ZippyDashboardIO[] m_ios;
	private final ZippyDashboardIOInputsAutoLogged[] m_inputs;
	private final ReefDisplayIO reefDisplayIO;
	private final ReefDisplayIOInputsAutoLogged reefDisplayInputs;

	/**
	 * Constructs a new ZippyDashboard.
	 */
	public ZippyDashboard(ReefDisplayIO displayIO, ZippyDashboardIO... ios)
	{
		m_ios = ios;
		m_inputs = new ZippyDashboardIOInputsAutoLogged[ios.length];
		for (int i = 0; i < ios.length; i++)
		{
			m_inputs[i] = new ZippyDashboardIOInputsAutoLogged();
		}
		this.reefDisplayIO = displayIO;
		this.reefDisplayInputs = new ReefDisplayIOInputsAutoLogged();
	}

    @AutoLogOutput
    public Pose2d getTargetPose()
    {
        return FieldConstants.REEF_POSITIONS.get(reefDisplayInputs.reefLocations);
    }

	@Override
	public void periodic()
	{
		for (int i = 0; i < m_ios.length; i++)
		{
			m_ios[i].updateInputs(m_inputs[i]);
			String name = m_ios.getClass().getSimpleName();
			if (name.contains("ZippyDashboardIO"))
				name = name.substring(name.indexOf("ZippyDashboardIO") + "ZippyDashboardIO".length());
			Logger.processInputs(getName() + "/" + name, m_inputs[i]);
		}
        reefDisplayIO.updateInputs(reefDisplayInputs);
        Logger.processInputs(getName() + "/ReefDisplayIO", reefDisplayInputs);
	}
}