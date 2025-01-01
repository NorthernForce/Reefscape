package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.Sendable;

public class ZippyDashboard
{
	private final ZippyDashboardIO m_io;
	private final ZippyDashboardIOInputsAutoLogged m_inputs = new ZippyDashboardIOInputsAutoLogged();

	public ZippyDashboard(ZippyDashboardIO io)
	{
		m_io = io;
	}

	public void publishValue(String key, Sendable value)
	{
		m_io.publishValue(key, value);
	}

	public void periodic()
	{
		m_io.updateInputs(m_inputs);
		Logger.processInputs("Dashboard/ZippyDashboard", m_inputs);
	}
}