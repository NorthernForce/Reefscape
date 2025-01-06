package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants
{
	public static Rotation2d getFieldRotation(Alliance alliance)
	{
		if (alliance == Alliance.Blue)
		{
			return Rotation2d.fromDegrees(0);
		} else
		{
			return Rotation2d.fromDegrees(180);
		}
	}
}
