package frc.robot.subsystems.reefscape;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.FieldConstants.ReefLocations;

public interface ReefDisplayIO
{
	@AutoLog
	public static class ReefDisplayIOInputs
	{
		public ReefLocations reefLocations = ReefLocations.A;
		public int level = 0;
		public boolean isConnected = false;
	}

	public default void setGrayedOut(ReefLocations reefLocation, int level, boolean grayedOut)
	{
	}

	public default void updateInputs(ReefDisplayIOInputs inputs)
	{
	}
}
