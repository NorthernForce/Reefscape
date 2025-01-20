package frc.robot.zippy.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.VisionConstants;

public class ZippyConstants
{
	public static class DrivetrainConstants
	{
		public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);
		public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.7);
		public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(3.0);
		public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(0.7);
	}

	private static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

	private static String camera0Name = "front_left_camera";
	private static String camera1Name = "front_right_camera";
	private static String camera2Name = "back_left_camera";
	private static String camera3Name = "back_right_camera";

	private static Transform3d robotToCamera0 = new Transform3d(Inches.of(13.731), Inches.of(13.731), Inches.of(11.248),
			new Rotation3d(Degrees.of(0.0), Degrees.of(10.0), Degrees.of(45.0)));
	private static Transform3d robotToCamera1 = new Transform3d(Inches.of(13.731), Inches.of(-13.731),
			Inches.of(11.248), new Rotation3d(Degrees.of(0.0), Degrees.of(10.0), Degrees.of(315.0)));
	private static Transform3d robotToCamera2 = new Transform3d(Inches.of(-13.731), Inches.of(13.731),
			Inches.of(11.248), new Rotation3d(Degrees.of(0.0), Degrees.of(10.0), Degrees.of(135)));
	private static Transform3d robotToCamera3 = new Transform3d(Inches.of(-13.731), Inches.of(-13.731),
			Inches.of(11.248), new Rotation3d(Degrees.of(0.0), Degrees.of(10.0), Degrees.of(225)));

	private static double maxAmbiguity = 0.3;
	private static double maxZError = 0.75;

	private static double linearStdDevBaseline = 0.02; // Meters
	private static double angularStdDevBaseline = 0.06; // Radians

	private static double[] cameraStdDevFactors = new double[]
	{ 1.0, // Camera 0
			1.0, // Camera 1
			1.0, // Camera 2
			1.0 // Camera3
	};

	private static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
	private static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

	public static VisionConstants visionConstants = new VisionConstants(aprilTagLayout, maxAmbiguity, maxZError,
			linearStdDevBaseline, angularStdDevBaseline, cameraStdDevFactors, linearStdDevMegatag2Factor,
			angularStdDevMegatag2Factor, new String[]
			{ camera0Name, camera1Name, camera3Name }, new Transform3d[]
			{ robotToCamera0, robotToCamera1, robotToCamera3 });
}
