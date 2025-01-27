package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

/**
 * @see frc.robot.subsystems.LedsIO
 * @param setColours    sets the colours of the LED
 * @param setOn         sets the LED on or off
 * @param setBrightness sets the brightness of the LED
 * @param updateInputs  updates the LED inputs
 * @param r             red value of the LED
 * @param g             green value of the LED
 * @param b             blue value of the LED
 */
public interface LedsIO
{
	@AutoLog
	public static class LedIOInputs
	{
		public boolean on = false;
		public int r, g, b;
		public double brightness;
		public int ledCount;
		public double speed;
		public boolean animating;
		public int animationIndex;
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

	public default void updateAnimating(boolean on)
	{
	}

	public default void rainbowAnimation(int ledCount, double speed, double brightness)
	{
	}

	public default void twinkleAnimation(int r, int g, int b, double speed)
	{
	}

	public default void colourFlow(int r, int g, int b, double speed, boolean direction, int offSet)
	{
	}

	public default void strobeAnimation(int r, int g, int b, double speed)
	{
	}

	public default void setSpecificLEDs(int startIdx, int endIdx, int r, int g, int b)
	{
	}

	public default void incrementAnimation()
	{
	}

	public default void clearAnimationBuffer()
	{
	}
}
