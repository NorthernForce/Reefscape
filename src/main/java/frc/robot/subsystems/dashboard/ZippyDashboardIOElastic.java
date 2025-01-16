package frc.robot.subsystems.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ZippyDashboardIOElastic implements ZippyDashboardIO
{

	public void publishValue(String key, Object value)
	{
		Shuffleboard.getTab("Zippy").add(key, value);
	}

	public void updateInputs(ZippyDashboardIOInputs inputs)
	{

	}

}
