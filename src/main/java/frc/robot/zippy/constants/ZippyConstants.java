package frc.robot.zippy.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ZippyConstants
{
	public static class DrivetrainConstants
	{
		public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);
		public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.7);
		public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(3.0);
		public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(0.7);
	}

	public static class AutoConstants
	{
		// TODO: tuning
		public static final PIDController xPID = new PIDController(10, 0, 0);
		public static final PIDController yPID = new PIDController(10, 0, 0);
		public static final PIDController rPID = new PIDController(7.5, 0, 0);
	}
}
