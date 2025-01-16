package frc.robot.subsystems.pneumatic;

import org.littletonrobotics.junction.AutoLog;

public interface PneumaticHubIO
{

	@AutoLog
	public static class PneumaticHubIOInputs
	{
		public boolean isAtPressure;
	}

	/**
	 * toggle functionality interface
	 */
	public default void toggle()
	{
	}

	/**
	 * enable interface functionality
	 */
	public default void enable()
	{
	}

	/**
	 * enable interface functionality
	 */
	public default void disable()
	{
	}

	/**
	 * update inputs for the Hub
	 * 
	 * @param inputs inputs to update with periodic
	 */
	public default void updateInputs(PneumaticHubIOInputs inputs)
	{
	}
}