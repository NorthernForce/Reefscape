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

	/*
	 * okay so correct me if I'm wrong Conner unlike Gyro I have seperate functions
	 * because Gyro updates all of the data in one go but led's values don't need to
	 * have all colour brightness and on or off all specified every time you change
	 * one value right?????
	 */
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
