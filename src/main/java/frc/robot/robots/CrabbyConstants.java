package frc.robot.robots;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
		public static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
		public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
		public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
		public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
		public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
		public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
	}

	public class VisionConstants
	{
		public static final Transform3d frontCameraTransform = new Transform3d(new Translation3d(0.0, 0.0, 0.0),
				new Rotation3d(0.0, 0.0, 0.0));
		public static final Transform3d backCameraTransform = new Transform3d(new Translation3d(0.0, 0.0, 0.0),
				new Rotation3d(0.0, 0.0, 0.0));
	}
}
