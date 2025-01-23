package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface IntakeIO
{

	@AutoLog
	public static class IntakeIOInputs
	{
		public Angle position = Rotations.of(0);
	}

	public void intake();

	public void outtake();

	public void updateInputs(IntakeIOInputs inputs);
}
