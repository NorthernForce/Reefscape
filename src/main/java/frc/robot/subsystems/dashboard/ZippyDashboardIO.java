package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;

public interface ZippyDashboardIO
{
	@AutoLog
	public static class ZippyDashboardIOInputs
	{
		public Pose2d pose = new Pose2d();
		public double voltage = 0.0;
		public double current = 0.0;
		public double refreshRate = 0.0;
	}

	public void publishValue(String key, Sendable value);

	public default void updateInputs(ZippyDashboardIOInputs inputs)
	{
	}
}
