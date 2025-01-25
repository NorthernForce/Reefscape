package frc.robot.subsystems.elevator.brake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.Relay.Value;

public interface BrakeIO
{
	@AutoLog
	public static class BrakeIOInputs
	{
		public Value isOn = Value.kOff;
	}

	public void setBreak(boolean on);

	public void setDirection(boolean clockwise);

	public default void updateInputs(BrakeIOInputs inputs)
	{
	}
}
