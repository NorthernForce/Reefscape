package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface CameraManagerIO
{

	@AutoLog
	public static class CameraManagerIOInputs
	{
		public EstimatedRobotPose[] estimatedPoses;
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(CameraManagerIOInputs inputs)
	{
	}
}
