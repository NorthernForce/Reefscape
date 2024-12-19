package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

public record VisionConstants(AprilTagFieldLayout aprilTagLayout, double maxAmbiguity, double maxZError,
		double linearStdDevBaseline, double angularStdDevBaseline, double[] cameraStdDevFactor,
		double linearStdDevMegatag2Factor, double angularStdDevMegatag2Factor, String[] cameraNames,
		Transform3d[] cameraTransforms) {
}
