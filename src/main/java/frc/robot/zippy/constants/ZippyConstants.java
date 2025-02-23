package frc.robot.zippy.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.blenny.constants.BlennyTunerConstants;

public class ZippyConstants
{
    public static class DrivetrainConstants
    {
        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.7);
        public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(4.0);
        public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(3.0);
        public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(0.7);
        public static final Angle[] SWERVE_MODULE_OFFSETS =
        { Rotations.of(Preferences.getDouble("kSwerveOffestFrontLeft", BlennyTunerConstants.FrontLeft.EncoderOffset)),
                Rotations.of(Preferences.getDouble("kSwerveOffestFrontRight",
                        BlennyTunerConstants.FrontRight.EncoderOffset)),
                Rotations.of(
                        Preferences.getDouble("kSwerveOffestBackLeft", BlennyTunerConstants.BackLeft.EncoderOffset)),
                Rotations.of(Preferences.getDouble("kSwerveOffestBackRight",
                        BlennyTunerConstants.BackRight.EncoderOffset)) };
    }

    public static class VisionConstants
    {
        public static final AprilTagFieldLayout APRILTAG_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeAndyMark);

        private static final String FL_CAMERA_NAME = "front_left_camera";
        private static final String FR_CAMERA_NAME = "front_right_camera";
        private static final String BL_CAMERA_NAME = "back_left_camera";
        private static final String BR_CAMERA_NAME = "back_right_camera";

        private static final Transform3d FL_ROBOT_TO_CAMERA = new Transform3d(Inches.of(13.731), Inches.of(13.731),
                Inches.of(11.248), new Rotation3d(Degrees.of(0.0), Degrees.of(10.0), Degrees.of(45.0)));
        private static final Transform3d FR_ROBOT_TO_CAMERA = new Transform3d(Inches.of(13.731), Inches.of(-13.731),
                Inches.of(11.248), new Rotation3d(Degrees.of(0.0), Degrees.of(10.0), Degrees.of(315.0)));
        private static final Transform3d BL_ROBOT_TO_CAMERA = new Transform3d(Inches.of(-13.731), Inches.of(13.731),
                Inches.of(11.248), new Rotation3d(Degrees.of(0.0), Degrees.of(10.0), Degrees.of(135)));
        private static final Transform3d BR_ROBOT_TO_CAMERA = new Transform3d(Inches.of(-13.731), Inches.of(-13.731),
                Inches.of(11.248), new Rotation3d(Degrees.of(0.0), Degrees.of(10.0), Degrees.of(225)));

        public static final double MAX_Y_COORDINATE = 350; // TODO: Set this to the actual value

        public static String[] cameraNames()
        {
            return new String[]
            { FL_CAMERA_NAME, FR_CAMERA_NAME, BL_CAMERA_NAME, BR_CAMERA_NAME };
        }

        public static Transform3d[] cameraTransforms()
        {
            return new Transform3d[]
            { FL_ROBOT_TO_CAMERA, FR_ROBOT_TO_CAMERA, BL_ROBOT_TO_CAMERA, BR_ROBOT_TO_CAMERA };
        }

        public static final double CAMERA_WIDTH = 800;
    }

    public static class AutoConstants
    {
        // TODO: tuning
        public static final PIDController xPID = new PIDController(10, 0, 0);
        public static final PIDController yPID = new PIDController(10, 0, 0);
        public static final PIDController rPID = new PIDController(7.5, 0, 0);
    }
}
