package frc.robot.blenny.constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.subsystems.superstructure.Superstructure.GenericSuperstructureGoal;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX.ElevatorConstants;
import frc.robot.subsystems.superstructure.wrist.WristIOTalonFX.WristConstants;

import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class BlennyConstants
{
    public static class DrivetrainConstants
    {

        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.7);
        public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(4.0);
        public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(3.0);
        public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(0.7);
        public static final Angle[] SWERVE_MODULE_OFFSETS =
        { Rotations.of(Preferences.getDouble("kSwerveOffsetFrontLeft", BlennyTunerConstants.FrontLeft.EncoderOffset)),
                Rotations.of(Preferences.getDouble("kSwerveOffsetFrontRight",
                        BlennyTunerConstants.FrontRight.EncoderOffset)),
                Rotations.of(
                        Preferences.getDouble("kSwerveOffsetBackLeft", BlennyTunerConstants.BackLeft.EncoderOffset)),
                Rotations.of(Preferences.getDouble("kSwerveOffsetBackRight",
                        BlennyTunerConstants.BackRight.EncoderOffset)) };
    }

    public static class VisionConstants
    {
        public static final AprilTagFieldLayout APRILTAG_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeAndyMark);

        private static final String FL_CAMERA_NAME = "front_left_camera";
        private static final String FR_CAMERA_NAME = "front_right_camera";

        private static final Transform3d FL_ROBOT_TO_CAMERA = new Transform3d(Inches.of(13.731), Inches.of(13.731),
                Inches.of(11.248), new Rotation3d(Degrees.of(0.0), Degrees.of(10.0), Degrees.of(45.0)));

        private static final Transform3d FR_ROBOT_TO_CAMERA = new Transform3d(Inches.of(13.731), Inches.of(-13.731),
                Inches.of(11.248), new Rotation3d(Degrees.of(0.0), Degrees.of(10.0), Degrees.of(315.0)));

        public static String[] cameraNames()
        {
            return new String[]
            { FL_CAMERA_NAME, FR_CAMERA_NAME };
        }

        public static Transform3d[] cameraTransforms()
        {
            return new Transform3d[]
            { FL_ROBOT_TO_CAMERA, FR_ROBOT_TO_CAMERA };
        }

        public static final double MAX_Y_COORDINATE = 350; // TODO: Set this to the actual value

        public static final double CAMERA_WIDTH = 800;
    }

    public static class InnerElevatorConstants
    {
        // outer ratios
        public static final double GEAR_BOX_RATIO = 25.0;
        public static final Distance SPROCKET_CIRCUMFERENCE = Inches.of(4.0);

        // talon configs
        public static final double kS = 0.12;
        public static final double kV = 0.7;
        public static final double kA = 0.5;
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double CRUISE_VELOCITY = 1;
        public static final double ACCELERATION = 0.008;
        public static final double JERK = 0;
        public static final double EXPO_kV = 0.12;
        public static final double EXPO_kA = 0.1;
        public static final Distance UPPER_LIMIT = Inches.of(25.8);

        public static final ElevatorConstants ELEVATOR_CONSTANTS = new ElevatorConstants(kS, kV, kA, kP, kI, kD,
                CRUISE_VELOCITY, ACCELERATION, JERK, EXPO_kV, EXPO_kA, SPROCKET_CIRCUMFERENCE, GEAR_BOX_RATIO, true,
                UPPER_LIMIT);

        public static final double HOMING_SPEED = 0.05;
    }

    public static class OuterElevatorConstants
    {
        // outer ratios
        public static final double GEAR_BOX_RATIO = 27.0;
        public static final Distance SPROCKET_CIRCUMFERENCE = Inches.of(4.5);

        // talon configs
        public static final double kS = 0.12;
        public static final double kV = 0.7;
        public static final double kA = 0.5;
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double CRUISE_VELOCITY = 1;
        public static final double ACCELERATION = 0.008;
        public static final double JERK = 0;
        public static final double EXPO_kV = 0.12;
        public static final double EXPO_kA = 0.1;
        public static final Distance UPPER_LIMIT = Inches.of(26.7);

        public static final ElevatorConstants ELEVATOR_CONSTANTS = new ElevatorConstants(kS, kV, kA, kP, kI, kD,
                CRUISE_VELOCITY, ACCELERATION, JERK, EXPO_kV, EXPO_kA, SPROCKET_CIRCUMFERENCE, GEAR_BOX_RATIO, false,
                UPPER_LIMIT);

        public static final double HOMING_SPEED = 0.05;
    }

    public static class WristJointConstants
    {
        public static final double kS = 0.2;
        public static final double kV = 20;
        public static final double kA = 30;
        public static final double kP = 15;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double CRUISE_VELOCITY = 700;
        public static final double ACCELERATION = 300;
        public static final double JERK = 600;
        public static final boolean INVERTED = false;
        public static final Angle UPPER_LIMIT = Rotations.of(0.098);
        public static final Angle LOWER_LIMIT = Degrees.of(0.252);
        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
        public static final double ROTOR_TO_SENSOR_RATIO = 192.0;
        public static final double MANUAL_MOVE_SPEED = 0.05;

        public static final WristConstants WRIST_CONSTANTS = new WristConstants(kS, kV, kA, kP, kI, kD, CRUISE_VELOCITY,
                ACCELERATION, JERK, INVERTED, UPPER_LIMIT, LOWER_LIMIT, SENSOR_TO_MECHANISM_RATIO,
                ROTOR_TO_SENSOR_RATIO);
    }

    /**
     * Superstructure states for the coral and algae
     */

    public static enum SuperstructureGoal implements GenericSuperstructureGoal
    {
        L1(Inches.of(0), Inches.of(0), Degrees.of(0)), L2(Inches.of(4.69), Inches.of(5.55), Rotations.of(-0.095)),
        L3(Inches.of(13.72), Inches.of(12.13), Rotations.of(-0.095)),
        L4(Inches.of(26.6), Inches.of(25.6), Rotations.of(-0.083)),
        CORAL_STATION(Inches.of(0), Inches.of(0), Degrees.of(0)),
        PROCESSOR_STATION(Inches.of(0), Inches.of(0), Degrees.of(0)),
        LOWER_ALGAE(Inches.of(0), Inches.of(0), Degrees.of(0)), HIGHER_ALGAE(Inches.of(0), Inches.of(0), Degrees.of(0));

        private final Distance innerHeight;
        private final Distance outerHeight;
        private final Angle wristAngle;

        /**
         * Superstructure state constructor
         * 
         * @param innerHeight height of the inner elevator to go to
         * @param outerHeight height of the outer elevator to go to
         * @param wristAngle  angle of the wrist to go to
         */

        private SuperstructureGoal(Distance innerHeight, Distance outerHeight, Angle wristAngle)
        {
            this.innerHeight = innerHeight;
            this.outerHeight = outerHeight;
            this.wristAngle = wristAngle;
        }

        @Override
        public Distance getInnerElevatorGoal()
        {
            return innerHeight;
        }

        @Override
        public Distance getOuterElevatorGoal()
        {
            return outerHeight;
        }

        @Override
        public Angle getWristGoal()
        {
            return wristAngle;
        }
    }

    public static class PathplannerConstants
    {
        public static final PIDConstants linearPIDConstants = new PIDConstants(10.0, 0.0, 0.0);
        public static final PIDConstants angularPIDConstants = new PIDConstants(5.0, 0.0, 0.0);
    }

    public static class ClimberConstants
    {
        public static final int ID = 17;
        public static final boolean INVERTED = false;
        public static final int ENCODER_ID = 23;
        public static final Angle LOWER_LIMIT = Rotations.of(-0.03);
        public static final Angle UPPER_LIMIT = Rotations.of(0.22);
    }

    public static class RollersConstants
    {
        public static final double INTAKE_SPEED = 0.8;
        public static final double OUTTAKE_SPEED = 0.8;
        public static final int ROLLER_MOTOR_LEFT_ID = 18;
        public static final int ROLLER_MOTOR_RIGHT_ID = 19;
        public static final boolean ROLLER_MOTORS_INVERTED = true;

        public static class SensorConstants
        {
            public static final int ULTRASONIC_CORAL_TRIGGER = 2;
            public static final int ULTRASONIC_CORAL_ECHO = 3;
            public static final int ULTRASONIC_ALGAE_TRIGGER = 4;
            public static final int ULTRASONIC_ALGAE_ECHO = 5;
            public static final Distance ULTRASONIC_CORAL_MAX_DISTANCE = Inches.of(2);
            public static final Distance ULTRASONIC_ALGAE_MAX_DISTANCE = Inches.of(2);
        }
    }
}
