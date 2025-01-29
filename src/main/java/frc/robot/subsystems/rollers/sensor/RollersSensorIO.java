package frc.robot.subsystems.rollers.sensor;

import org.littletonrobotics.junction.AutoLog;

/**
 * The IO for the rollers algae and coral sensor.
 */

public interface RollersSensorIO
{

	/**
	 * The inputs for the rollers algae and coral sensor.
	 */

	@AutoLog
	public static class RollersSensorIOInputs
	{
		public boolean hasPiece;
	}

	/**
	 * Updates the inputs for the rollers algae and coral sensor.
	 * 
	 * @param inputs The inputs to update.
	 */

	public void updateInputs(RollersSensorIOInputs inputs);
}
