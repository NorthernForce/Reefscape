package frc.robot.blenny.constants;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.*;

public class BlennyConstants
{
    public static class DrivetrainConstants
    {

        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.7);
        public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(3.0);
        public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(0.7);
    }

    public static class RollersConstants
    {
        public static final double INTAKE_SPEED = 1;
        public static final double OUTTAKE_SPEED = 1;
        public static final int ROLLER_MOTOR_ONE_ID = 30;
        public static final int ROLLER_MOTOR_TWO_ID = 31;
        public static final boolean ROLLER_MOTORS_INVERTED = false;

        public static class SensorConstants
        {
            public static final int ULTRASONIC_ONE_TRIGGER = 0;
            public static final int ULTRASONIC_ONE_ECHO = 1;
            public static final int ULTRASONIC_TWO_TRIGGER = 2;
            public static final int ULTRASONIC_TWO_ECHO = 3;
            public static final double ULTRASONIC_ONE_MAX_DISTANCE = 50;
            public static final double ULTRASONIC_TWO_MAX_DISTANCE = 50;
        }
    }
}
