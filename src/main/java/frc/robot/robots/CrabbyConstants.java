package frc.robot.robots;

import edu.wpi.first.math.util.Units;

public class CrabbyConstants
{
	public class DriveConstants
	{
		public static final double driveFeedforwardks = 0.10;
		public static final double driveFeedforwardkv = 0.13;
		public static final double driveFeedbackkp = 0.05;
		public static final double driveFeedbackki = 0.00;
		public static final double driveFeedbackkd = 0.00;

		public static final double turnFeedbackkp = 7.0;
		public static final double turnFeedbackki = 0.0;
		public static final double turnFeedbackkd = 0.0;

		public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
		public static final double TRACK_WIDTH_X = 0.45085;
		public static final double TRACK_WIDTH_Y = 0.61595;
		public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
		public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
		public static final double DRIVE_GEAR_RATIO = 6.75;
		public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
	}

}
