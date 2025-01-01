package frc.robot.subsystems.dashboard;

public interface ZippyDashboard
{
	public void publishValue(String key, Object value);

	public void destroyValue(String key);
}
