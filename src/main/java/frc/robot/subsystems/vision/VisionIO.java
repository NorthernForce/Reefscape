package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO
{
	@AutoLog
	public static class VisionIOInputs
	{
		public boolean connected = false;
		public TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
		public PoseObservation[] poseObservations = new PoseObservation[0];
		public int[] tagIds = new int[0];
	}

	public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
	}

	public static record PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount,
			double averageTagDistance, PoseObservationType type) {
	}

	public static enum PoseObservationType
	{
		MEGATAG_1, MEGATAG_2, PHOTONVISION
	}

	public default int getNumberOfCameras()
	{
		return 0;
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(VisionIOInputs inputs)
	{
	}
}
