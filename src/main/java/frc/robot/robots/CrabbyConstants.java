package frc.robot.robots;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.ModuleIOTalonFX.ModuleIOTalonFXParameters;

public class CrabbyConstants
{
	public class DriveConstants
	{
		public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
		public static final double TRACK_WIDTH_X = 0.45085;
		public static final double TRACK_WIDTH_Y = 0.61595;
		public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
		public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
		public static final double DRIVE_GEAR_RATIO = 6.75;
		public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

		public static final double DRIVE_CURRENT_LIMIT = 40.0;
		public static final double TURN_CURRENT_LIMIT = 40.0;

		public static final double DRIVE_P = 0.1;
		public static final double DRIVE_I = 0.01;
		public static final double DRIVE_V = 0.1;

		public static final double TURN_P = 0.1;
		public static final double TURN_D = 0.01;

		public static final double ODOMETRY_FREQUENCY = 250.0;

		public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

		public static final ModuleIOTalonFXParameters MODULE_PARAMETERS = new ModuleIOTalonFXParameters(TURN_GEAR_RATIO,
				DRIVE_GEAR_RATIO, DRIVE_CURRENT_LIMIT, TURN_CURRENT_LIMIT, ODOMETRY_FREQUENCY, WHEEL_RADIUS, DRIVE_P,
				DRIVE_I, DRIVE_V, TURN_P, TURN_D);
	}

}
