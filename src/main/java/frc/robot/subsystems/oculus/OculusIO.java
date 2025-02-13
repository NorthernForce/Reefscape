package frc.robot.subsystems.oculus;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface OculusIO
{
	public default Pose2d getPose()
	{
		return new Pose2d();
	}

	public default double getBatteryPercent()
	{
		return 0.0;
	}

	public default boolean isConnected()
	{
		return false;
	}

	public default Quaternion getOrientation()
	{
		return new Quaternion();
	}

	public default double timestamp()
	{
		return 0.0;
	}

	public default void zeroHeading()
	{
	}

	public default void zeroPosition()
	{
	}

	public default void cleanUpQuestNavMessages()
	{
	}

	public default float getOculusYaw()
	{
		return 0.0f;
	}

	public default Translation2d getQuestNavTranslation()
	{
		return new Translation2d();
	}

	public default Pose2d getQuestNavPose()
	{
		return new Pose2d();
	}
}
