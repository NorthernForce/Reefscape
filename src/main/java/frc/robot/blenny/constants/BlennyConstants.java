package frc.robot.blenny.constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.superstructure.Superstructure.GenericSuperstructureGoal;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class BlennyConstants
{
    public static class DrivetrainConstants
    {
        /** TODO */
        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.7);
        public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(3.0);
        public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(0.7);
    }

    public static class VisionConstants
    {
        public static final AprilTagFieldLayout APRILTAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

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
    }

    public static class InnerElevatorConstants
    {
        // outer ratios
        public static final double GEAR_BOX_RATIO = 20.0;
        public static final double SPROCKET_DIAM = 1.44;
        public static final double SPROCKET_TEETH = 18;
        public static final double GEAR_RATIO = GEAR_BOX_RATIO * SPROCKET_TEETH / SPROCKET_DIAM;
        public static final double SPROCKET_CIRCUMFERENCE = Math.PI * SPROCKET_DIAM;

        // talon configs
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.02;
        public static final double kP = 4.8;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final double CRUISE_VELOCITY = 80;
        public static final double ACCELERATION = 160;
        public static final double JERK = 1600;
        public static final Distance UPPER_LIMIT = Inches.of(0);

        public static final ElevatorConstants ELEVATOR_CONSTANTS = new ElevatorConstants(kS, kV, kA, kP, kI, kD,
                MetersPerSecond.of(CRUISE_VELOCITY), MetersPerSecondPerSecond.of(ACCELERATION), JERK,
                Meters.of(SPROCKET_CIRCUMFERENCE), GEAR_RATIO, false, UPPER_LIMIT);
    }

    public static class OuterElevatorConstants
    {
        // outer ratios
        public static final double GEAR_BOX_RATIO = 20.0;
        public static final double SPROCKET_DIAM = 1.44;
        public static final double SPROCKET_TEETH = 18;
        public static final double GEAR_RATIO = GEAR_BOX_RATIO * SPROCKET_TEETH / SPROCKET_DIAM;
        public static final double SPROCKET_CIRCUMFERENCE = Math.PI * SPROCKET_DIAM;

        // talon configs
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.02;
        public static final double kP = 4.8;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final double CRUISE_VELOCITY = 80;
        public static final double ACCELERATION = 160;
        public static final double JERK = 1600;
        public static final Distance UPPER_LIMIT = Inches.of(0);

        public static final ElevatorConstants ELEVATOR_CONSTANTS = new ElevatorConstants(kS, kV, kA, kP, kI, kD,
                MetersPerSecond.of(CRUISE_VELOCITY), MetersPerSecondPerSecond.of(ACCELERATION), JERK,
                Meters.of(SPROCKET_CIRCUMFERENCE), GEAR_RATIO, false, UPPER_LIMIT);
    }

    /**
     * Superstructure states for the coral and algae
     */

    public static enum SuperstructureGoal implements GenericSuperstructureGoal
    {
        L1(Inches.of(0), Inches.of(0), Degrees.of(0)), L2(Inches.of(0), Inches.of(0), Degrees.of(0)),
        L3(Inches.of(0), Inches.of(0), Degrees.of(0)), L4(Inches.of(0), Inches.of(0), Degrees.of(0)),
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
}
