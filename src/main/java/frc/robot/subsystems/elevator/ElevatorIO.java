package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO
{
	@AutoLog
	public static class ElevatorIOInputs
	{
		public double temperature;
		public ElevatorState state;
	}

	public void start(double speed);

	public void stop();

	public Distance getPosition();

	public void updateInputs(ElevatorIOInputs inputs);

	public void setInverted(boolean inverted);
}
