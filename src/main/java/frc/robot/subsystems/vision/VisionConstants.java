package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants
{
	public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

	public static String camera0Name = "camera_0";
	public static String camera1Name = "camera_1";
	public static String camera2Name = "camera_2";
	public static String camera3Name = "camera_3";

	public static Transform3d robotToCamera0 = new Transform3d(13.731, 13.731, 11.248,
			new Rotation3d(0, Math.toRadians(10), Math.toRadians(315)));
	public static Transform3d robotToCamera1 = new Transform3d(13.731, -13.731, 11.248,
			new Rotation3d(0, Math.toRadians(10), Math.toRadians(225)));
	public static Transform3d robotToCamera2 = new Transform3d(-13.731, -13.731, 11.248,
			new Rotation3d(0, Math.toRadians(10), Math.toRadians(135)));
	public static Transform3d robotToCamera3 = new Transform3d(-13.731, 13.731, 11.248,
			new Rotation3d(0, Math.toRadians(10), Math.toRadians(45)));

	public static double maxAmbiguity = 0.3;
	public static double maxZError = 0.75;

	public static double linearStdDevBaseline = 0.02; // Meters
	public static double angularStdDevBaseline = 0.06; // Radians

	public static double[] cameraStdDevFactor = new double[]
	{ 1.0, // Camera 0
			1.0, // Camera 1
			1.0, // Camera 2
			1.0 // Camera 3
	};

	public static double linearStdDevMegatag2Factor = 0.5;
	public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}
