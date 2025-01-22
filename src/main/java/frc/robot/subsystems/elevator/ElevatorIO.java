package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO
{
	@AutoLog
	public static class ElevatorIOInputs
	{
		public double temperature = 0;
		public Distance position = Meters.of(0);
		public boolean isAtTargetPosition = false;
		public ElevatorState goingTo = ElevatorState.L1;
	}

	public void start(double speed, ElevatorState level);

	public void stop();

	public void updateInputs(ElevatorIOInputs inputs);

	public void setInverted(boolean inverted);
}
