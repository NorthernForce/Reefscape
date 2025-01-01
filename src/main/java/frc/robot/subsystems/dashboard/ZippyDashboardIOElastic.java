package frc.robot.subsystems.dashboard;

import java.time.Instant;
import java.util.Date;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ZippyDashboardIOElastic implements ZippyDashboardIO
{
	private DoubleSupplier time = () -> Date.from(Instant.now()).getTime();
	private long timestamp = (long) time.getAsDouble();

	public void publishValue(String key, Sendable value)
	{
		SmartDashboard.putData(key, value);
	}

	public void updateInputs(ZippyDashboardIOInputs inputs)
	{

		inputs.pose = new Pose2d(
				new Translation2d(SmartDashboard.getNumber("poseX", 0.0), SmartDashboard.getNumber("poseY", 0.0)),
				new Rotation2d(SmartDashboard.getNumber("poseTheta", 0.0)));
		inputs.voltage = SmartDashboard.getNumber("voltage", 0.0);
		inputs.current = SmartDashboard.getNumber("current", 0.0);
		inputs.refreshRate = 1000.0 / ((long) time.getAsDouble() - timestamp); // fps
		timestamp = (long) time.getAsDouble();
	}

}
