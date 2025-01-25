package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO
{
	@AutoLog
	public static class ClimberIOInputs
	{
		public Angle position = Rotations.of(0);
	}

	public void climbUp();

	public void climbDown();

	public void stop();

	public void updateInputs(ClimberIOInputs inputs);
}