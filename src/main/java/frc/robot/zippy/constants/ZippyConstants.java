package frc.robot.zippy.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ZippyConstants
{
	public static class DrivetrainConstants
	{
		public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);
		public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.7);
		public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(3.0);
		public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(0.7);
	}

	public static class FieldConstants // TODO: Change these values
	{
		public static final double L1_HEIGHT = 1;
		public static final double L2_HEIGHT = 3;
		public static final double L3_HEIGHT = 5;
		public static final double L4_HEIGHT = 8;
	}

	public static class ElevatorConstants
	{
        // outer ratios
		public static final double GEAR_BOX_RATIO = 20.0;
		public static final double SPROCKET_DIAM = 1.44;
		public static final double SPROCKET_TEETH = 18;
		public static final double GEAR_RATIO = GEAR_BOX_RATIO * SPROCKET_TEETH / SPROCKET_DIAM;
		public static final double SPROCKET_CIRCUMFERENCE = Math.PI * SPROCKET_DIAM;

        // inner ratios
        public static final double INNER_GEAR_BOX_RATIO = 20.0;
        public static final double INNER_SPROCKET_DIAM = 1.44;
        public static final double INNER_SPROCKET_TEETH = 18;
        public static final double INNER_GEAR_RATIO = INNER_GEAR_BOX_RATIO * INNER_SPROCKET_TEETH / INNER_SPROCKET_DIAM;
        public static final double INNER_SPROCKET_CIRCUMFERENCE = Math.PI * INNER_SPROCKET_DIAM;
	}
}
