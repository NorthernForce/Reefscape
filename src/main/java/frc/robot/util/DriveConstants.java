package frc.robot.util;

public record DriveConstants(double odometryFrequency, double maxLinearSpeed, double trackWidthX, double trackWidthY,
		double driveBaseRadius, double maxAngularSpeed, double mk4GearRatio, double kDriveGearRatio,
		double kSteerGearRatio, double kWheelRadius, double kWheelCircumference, double kDriveP, double kTurnP,
		double kMaxDriveSpeed) {
}