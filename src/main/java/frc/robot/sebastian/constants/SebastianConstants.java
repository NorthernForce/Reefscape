package frc.robot.sebastian.constants;

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class SebastianConstants
{
    public static class DrivetrainConstants
    {

        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.7);
        public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(4.0);
        public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(3.0);
        public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(0.7);
        public static final Distance SAFE_DISTANCE = Inches.of(10);
        public static final Angle[] SWERVE_MODULE_OFFSETS =
        { Rotations
                .of(Preferences.getDouble("kSwerveOffsetFrontLeft", SebastianTunerConstants.FrontLeft.EncoderOffset)),
                Rotations.of(Preferences.getDouble("kSwerveOffsetFrontRight",
                        SebastianTunerConstants.FrontRight.EncoderOffset)),
                Rotations.of(
                        Preferences.getDouble("kSwerveOffsetBackLeft", SebastianTunerConstants.BackLeft.EncoderOffset)),
                Rotations.of(Preferences.getDouble("kSwerveOffsetBackRight",
                        SebastianTunerConstants.BackRight.EncoderOffset)) };
        /**
         * The maximum acceleration of the robot in duty cycles per second squared. Only
         * when the robot's outer elevator is above 4 inches or the inner elevator is
         * above 10 inches.
         */
        public static final double SLOW_RATE = 0.5;
    }

    public static class VisionConstants
    {
        public static final AprilTagFieldLayout APRILTAG_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeAndyMark);

        private static final String FL_CAMERA_NAME = "front_left_camera";
        private static final String FR_CAMERA_NAME = "front_right_camera";
        private static final String BL_CAMERA_NAME = "back_left_camera";
        private static final String BR_CAMERA_NAME = "back_right_camera";

        private static final Transform3d FL_ROBOT_TO_CAMERA = new Transform3d(Inches.of(11.5), Inches.of(7),
                Inches.of(8.5), new Rotation3d(Degrees.of(20.75), Degrees.of(15.0), Degrees.of(45.0)));

        private static final Transform3d FR_ROBOT_TO_CAMERA = new Transform3d(Inches.of(11.5), Inches.of(-7.0),
                Inches.of(8.5), new Rotation3d(Degrees.of(-20.75), Degrees.of(15.0), Degrees.of(315.0)));

        private static final Transform3d BL_ROBOT_TO_CAMERA = new Transform3d(Inches.of(-13.5), Inches.of(7),
                Inches.of(8.5), new Rotation3d(Degrees.of(0.0), Degrees.of(15.0), Degrees.of(135.0)));

        private static final Transform3d BR_ROBOT_TO_CAMERA = new Transform3d(Inches.of(-13.5), Inches.of(-7.0),
                Inches.of(8.5), new Rotation3d(Degrees.of(0.0), Degrees.of(15.0), Degrees.of(225.0)));

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

        public static final double MAX_Y_COORDINATE = 350; // TODO: Set this to the actual value

        public static final double CAMERA_WIDTH = 800;
    }

    public static class InnerElevatorConstants
    {
        // outer ratios
        public static final double GEAR_BOX_RATIO = 25.0;
        public static final double SPROCKET_TEETH = 16.0;
        public static final Distance SPROCKET_PITCH = Inches.of(0.25);
        public static final Distance SPROCKET_CIRCUMFERENCE = SPROCKET_PITCH.times(SPROCKET_TEETH);

        // talon configs
        public static final double kS = 0.12;
        public static final double kV = 0.67;
        public static final double kA = 0.2;
        public static final double kP = 12.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kG = 0;
        public static final double CRUISE_VELOCITY = 160;
        public static final double ACCELERATION = 60;
        public static final double JERK = 200;
        public static final Distance UPPER_LIMIT = Inches.of(25.8 - 2.75);

        public static final ElevatorConstants ELEVATOR_CONSTANTS = new ElevatorConstants(kS, kV, kA, kP, kI, kD, kG,
                CRUISE_VELOCITY, ACCELERATION, JERK, SPROCKET_CIRCUMFERENCE, GEAR_BOX_RATIO, true, UPPER_LIMIT);

        public static final double HOMING_SPEED = 0.25;

        public static final Distance HIGH_POSITION = Inches.of(10);

        public static final Distance TOLERANCE = Inches.of(0.5);
    }

    public static class OuterElevatorConstants
    {
        // outer ratios
        public static final double GEAR_BOX_RATIO = 25.0;
        public static final double SPROCKET_TEETH = 22.0;
        public static final Distance SPROCKET_PITCH = Inches.of(0.25);
        public static final Distance SPROCKET_CIRCUMFERENCE = SPROCKET_PITCH.times(SPROCKET_TEETH);

        // talon configs
        public static final double kS = 0.12;
        public static final double kV = 0.67;
        public static final double kA = 0.2;
        public static final double kP = 12.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kG = 0.234;
        public static final double CRUISE_VELOCITY = 160;
        public static final double ACCELERATION = 60;
        public static final double JERK = 299;
        public static final Distance UPPER_LIMIT = Inches.of(27.3);

        public static final ElevatorConstants ELEVATOR_CONSTANTS = new ElevatorConstants(kS, kV, kA, kP, kI, kD, kG,
                CRUISE_VELOCITY, ACCELERATION, JERK, SPROCKET_CIRCUMFERENCE, GEAR_BOX_RATIO, false, UPPER_LIMIT);

        public static final double HOMING_SPEED = 0.25;

        public static final Distance HIGH_POSITION = Inches.of(4);

        public static final Distance TOLERANCE = Inches.of(0.5);
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
        public static final Angle LOWER_LIMIT = Rotations.of(-0.252);
        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
        public static final double ROTOR_TO_SENSOR_RATIO = 192.0;
        public static final double MANUAL_MOVE_SPEED = 0.2;

        public static final WristConstants WRIST_CONSTANTS = new WristConstants(kS, kV, kA, kP, kI, kD, CRUISE_VELOCITY,
                ACCELERATION, JERK, INVERTED, UPPER_LIMIT, LOWER_LIMIT, SENSOR_TO_MECHANISM_RATIO,
                ROTOR_TO_SENSOR_RATIO);

        public static final Angle WRIST_TOLERANCE = Degrees.of(2);
    }

    /**
     * Superstructure states for the coral and algae
     */

    public static enum SuperstructureGoal implements GenericSuperstructureGoal
    {
        L1(Inches.of(0), Inches.of(0), Degrees.of(-0.03)),
        L2(Inches.of(13.1 - 2.75), Inches.of(0), Rotations.of(-0.095)),
        L3(Inches.of(25.8 - 2.75), Inches.of(4.61), Rotations.of(-0.095)),
        L4(Inches.of(27.3 - 2.75), Inches.of(26.6), Rotations.of(-0.059)),
        CORAL_STATION(Inches.of(7.11 - 2.75), Inches.of(0), WristJointConstants.UPPER_LIMIT),
        PROCESSOR_STATION(Inches.of(0), Inches.of(0), Degrees.of(0)),
        LOWER_ALGAE(Inches.of(0), Inches.of(0), Degrees.of(0)), HIGHER_ALGAE(Inches.of(0), Inches.of(0), Degrees.of(0)),
        START(Inches.of(0), Inches.of(0), WristJointConstants.LOWER_LIMIT),
        STOW_ALGAE(Inches.of(0), Inches.of(0), WristJointConstants.LOWER_LIMIT);

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
        public static final LinearVelocity MAX_VELOCITY = FeetPerSecond.of(4);
        public static final LinearAcceleration MAX_ACCELERATION = FeetPerSecondPerSecond.of(3);
        public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.7);
        public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(0.7);
    }

    public static class ClimberConstants
    {
        public static final int ID = 17;
        public static final boolean INVERTED = false;
        public static final int ENCODER_ID = 23;
        public static final Angle LOWER_LIMIT = Rotations.of(-0.03);
        public static final Angle UPPER_LIMIT = Rotations.of(0.22);
        public static final Angle SWEET_ANGLE = Degrees.of(70.0);
        public static final double CLIMB_SPEED = 1;
    }

    public static class RollersConstants
    {
        public static final double INTAKE_SPEED = 0.5;
        public static final double OUTTAKE_SPEED = 0.3;
        public static final int ROLLER_MOTOR_LEFT_ID = 18;
        public static final int ROLLER_MOTOR_RIGHT_ID = 19;
        public static final boolean ROLLER_MOTORS_INVERTED = false;

        public static class SensorConstants
        {
            public static final int ULTRASONIC_CORAL_TRIGGER = 2;
            public static final int ULTRASONIC_CORAL_ECHO = 3;
            public static final int ULTRASONIC_ALGAE_TRIGGER = 4;
            public static final int ULTRASONIC_ALGAE_ECHO = 5;
            public static final Distance CORAL_MAX_DISTANCE = Inches.of(2);
            public static final Distance ALGAE_MAX_DISTANCE = Inches.of(2);
            public static final int ANALOG_CORAL = 2;
            public static final int ANALOG_ALGAE = 1;
        }
    }

    public static class AutoConstants
    {
        // TODO: tuning
        public static final PIDController xPID = new PIDController(10, 0, 0);
        public static final PIDController yPID = new PIDController(10, 0, 0);
        public static final PIDController rPID = new PIDController(7.5, 0, 0);
    }
}
