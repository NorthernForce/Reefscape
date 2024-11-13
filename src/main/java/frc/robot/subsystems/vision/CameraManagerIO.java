package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface CameraManagerIO
{

	@AutoLog
	public static class CameraManagerIOInputs
	{
		public ArrayList<EstimatedRobotPose> estimatedPoses;
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(CameraManagerIOInputs inputs)
	{
	}
}
