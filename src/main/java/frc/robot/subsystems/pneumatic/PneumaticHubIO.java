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
     * get the hardware interface name (even though we ONLY USE REV CONNOR (😡))
     */
    public String getName();

    /**
     * update inputs for the Hub
     * @param inputs inputs to update with periodic
     */
	public default void updateInputs(PneumaticHubIOInputs inputs)
	{
	}
}