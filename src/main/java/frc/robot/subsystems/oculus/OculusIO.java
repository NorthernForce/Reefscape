package frc.robot.subsystems.oculus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface OculusIO
{

	public static class OculusIOInputs
	{
		public boolean connected = false;
		public Translation2d getOculusPosition = new Translation2d();
		public float getOculusYaw = 0;
		public Pose2d getOculusPose = new Pose2d();
	}

	public default void updateInputs(OculusIOInputs inputs)
	{
	}
}
