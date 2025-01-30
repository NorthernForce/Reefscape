package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO for the climber.
 */

public interface ClimberIO
{
	/**
	 * ClimberIOInputs class.
	 */

	@AutoLog
	public static class ClimberIOInputs
	{
		public Angle position = Rotations.of(0);
		public boolean present = false;
		public Temperature temperature = Fahrenheit.of(0);
		public Current current = Amps.of(0);
	}

	/**
	 * climb up method for the ClimberIO class.
	 * 
	 * @param climbSpeed speed to climb up
	 */

	public void climbUp(double climbSpeed);

	/**
	 * climb down method for the ClimberIO class.
	 * 
	 * @param climbSpeed speed to climb down
	 */

	public void climbDown(double climbSpeed);

	/**
	 * get climb up command method for the ClimberIO class.
	 * 
	 * @param climbSpeed speed to climb up
	 * @return command to climb up
	 */

	public void stop();

	/**
	 * update inputs method for the ClimberIO class.
	 * 
	 * @param inputs inputs for the climber
	 */

	public void updateInputs(ClimberIOInputs inputs);
}