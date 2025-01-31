package frc.robot.blenny.constants;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.config.PIDConstants;

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

    public static class PathplannerConstants
    {
        public static final PIDConstants linearPIDConstants = new PIDConstants(10.0, 0.0, 0.0);
        public static final PIDConstants angularPIDConstants = new PIDConstants(5.0, 0.0, 0.0);
    }
}
