package frc.robot.subsystems.rollers.sensor;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO
{

	@AutoLog
	public static class SensorIOInputs
	{
		public boolean hasPiece;
	}

	public void updateInputs(SensorIOInputs inputs);
}
