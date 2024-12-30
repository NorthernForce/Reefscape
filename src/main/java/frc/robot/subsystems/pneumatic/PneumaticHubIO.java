package frc.robot.subsystems.pneumatic;

import org.littletonrobotics.junction.AutoLog;

public interface PneumaticHubIO
{

	@AutoLog
	public static class PneumaticHubIOInputs
	{
		public boolean isAtPressure;
	}

	public default void toggle()
	{
	}

	public default void enable()
	{
	}

	public default void disable()
	{
	}

	public default void updateInputs(PneumaticHubIOInputs inputs)
	{
	}
}