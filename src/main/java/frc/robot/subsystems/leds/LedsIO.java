package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LedsIO
{
	@AutoLog
	public static class LedIOInputs
	{
		public boolean on = false;
		public int r, g, b;
		public double brightness;
	}

	public default void setColours(int r, int g, int b)
	{
	}

	public default void setOn(boolean on)
	{
	}

	public default void setBrightness(double brightness)
	{
	}

	public default void updateInputs(LedIOInputs inputs)
	{
	}
}
