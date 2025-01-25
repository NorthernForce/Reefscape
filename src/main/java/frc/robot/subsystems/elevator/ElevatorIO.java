package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import frc.robot.zippy.constants.ZippyConstants.ElevatorConstants.ElevatorState;

public interface ElevatorIO
{
	@AutoLog
	public static class ElevatorIOInputs
	{
		public double temperature = 0;
		public Distance position = Meters.of(0);
		public boolean isAtTargetPosition = false;
	}

	public default void setTargetPosition(double speed, ElevatorState level)
	{
	}

	public default void stop()
	{
	}

	public default void updateInputs(ElevatorIOInputs inputs)
	{
	}

	public default void setInverted(boolean inverted)
	{
	}
}
