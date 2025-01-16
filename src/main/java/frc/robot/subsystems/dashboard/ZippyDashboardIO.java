package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLog;

public interface ZippyDashboardIO
{
	@AutoLog
	public static class ZippyDashboardIOInputs
	{

	}

	public void publishValue(String key, Object value);

	public default void updateInputs(ZippyDashboardIOInputs inputs)
	{
	}
}
