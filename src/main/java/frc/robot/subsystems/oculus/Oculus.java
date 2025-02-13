package frc.robot.subsystems.oculus;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class Oculus extends SubsystemBase
{
	private final OculusIO io;

	public Oculus(OculusIO io)
	{
		this.io = io;
	}

	@Override
	public void periodic()
	{
	}

	public Pose2d getPose()
	{
		return io.getPose();
	}

	public double getBatteryPercent()
	{
		return io.getBatteryPercent();
	}

	public boolean isConnected()
	{
		return io.isConnected();
	}

	public Quaternion getOrientation()
	{
		return io.getOrientation();
	}

	public double timestamp()
	{
		return io.timestamp();
	}

	public void zeroHeading()
	{
		io.zeroHeading();
	}

	public void zeroPosition()
	{
		io.zeroPosition();
	}

	public void cleanUpQuestNavMessages()
	{
		io.cleanUpQuestNavMessages();
	}

	public float getOculusYaw()
	{
		return io.getOculusYaw();
	}

	public Translation2d getQuestNavTranslation()
	{
		return io.getQuestNavTranslation();
	}

	public Pose2d getQuestNavPose()
	{
		return io.getQuestNavPose();
	}
}
