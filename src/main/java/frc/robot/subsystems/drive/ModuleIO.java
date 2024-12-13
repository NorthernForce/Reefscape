package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO
{
	@AutoLog
	public static class ModuleIOInputs
	{
		public double drivePositionMeters = 0.0;
		public double driveVelocityMetersPerSecond = 0.0;

		public double turnVelocityRotationsPerSecond = 0.0;

		public double[] odometryDrivePositionsMeters = new double[]
		{};

		public double driveTemperature = 0.0;
		public double turnTemperature = 0.0;
		public boolean driveConnected = false;
		public double drivePositionRad = 0.0;
		public double driveVelocityRadPerSec = 0.0;
		public double driveAppliedVolts = 0.0;
		public double driveCurrentAmps = 0.0;

		public boolean turnConnected = false;
		public boolean turnEncoderConnected = false;
		public Rotation2d turnAbsolutePosition = new Rotation2d();
		public Rotation2d turnPosition = new Rotation2d();
		public double turnVelocityRadPerSec = 0.0;
		public double turnAppliedVolts = 0.0;
		public double turnCurrentAmps = 0.0;

		public double[] odometryTimestamps = new double[]
		{};
		public double[] odometryDrivePositionsRad = new double[]
		{};
		public Rotation2d[] odometryTurnPositions = new Rotation2d[]
		{};

	}

	public default void setDriveVoltage(double volts)
	{
	}

	public default void setTurnVoltage(double volts)
	{
	}

	public default void setDriveBrakeMode(boolean enable)
	{
	}

	public default void setTurnBrakeMode(boolean enable)
	{
	}

	public default void updateInputs(ModuleIOInputs inputs)
	{
	}

	public default void setDriveOpenLoop(double output)
	{
	}

	public default void setTurnOpenLoop(double output)
	{
	}

	public default void setDriveVelocity(double velocityRadPerSec)
	{
	}

	public default void setTurnPosition(Rotation2d rotation)
	{
	}

}
