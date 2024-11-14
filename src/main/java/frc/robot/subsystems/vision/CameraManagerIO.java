package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface CameraManagerIO
{

	@AutoLog
	public static class CameraManagerIOInputs
	{
		// public boolean[] connectedCameras;
		public Pose2d[] poses;
		public double[] timestamps;
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(CameraManagerIOInputs inputs)
	{
	}
}
