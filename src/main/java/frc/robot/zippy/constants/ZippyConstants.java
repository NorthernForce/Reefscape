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

		public static enum ElevatorState
		{
			L1(1, 0, 0), L2(2, 5, 0), L3(3, 0, 5), L4(4, 5, 5);

			private int level;
			private double innerHeight;
			private double outerHeight;

			ElevatorState(int level, double innerHeight, double outerHeight)
			{
				this.level = level;
				this.outerHeight = outerHeight;
				this.innerHeight = innerHeight;
			}

			public double getLevel()
			{
				return level;
			}

			public double getOuterHeight()
			{
				return outerHeight;
			}

			public double getInnerHeight()
			{
				return innerHeight;
			}
		}

	}
}
