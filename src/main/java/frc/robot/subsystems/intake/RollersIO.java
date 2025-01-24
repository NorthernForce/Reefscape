package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Temperature;

public interface RollersIO
{

	@AutoLog
	public static class IntakeIOInputs
	{
		public Temperature temperature;
	}

	public void set(double speed);

	public void updateInputs(IntakeIOInputs inputs);
}
