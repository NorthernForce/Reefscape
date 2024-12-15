package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.units.measure.*;

public record TunerConstants(Slot0Configs steerGains, Slot0Configs driveGains,
		ClosedLoopOutputType kSteerClosedLoopOutput, ClosedLoopOutputType kDriveClosedLoopOutput,
		SteerFeedbackType kSteerFeedbackType, Current kSlipCurrent, TalonFXConfiguration driveInitialConfigs,
		TalonFXConfiguration steerInitialConfigs, CANcoderConfiguration cancoderInitialConfigs,
		Pigeon2Configuration pigeonConfigs, CANBus kCANBus, LinearVelocity kSpeedAt12Volts, double kCoupleRatio,
		double kDriveGearRatio, double kSteerGearRatio, Distance kWheelRadius, boolean kInvertLeftSide,
		boolean kInvertRightSide, int kPigeonId, double kSteerInertia, double kDriveInertia,
		Voltage kSteerFrictionVoltage, Voltage kDriveFrictionVoltage, SwerveDrivetrainConstants DrivetrainConstants,
		SwerveModuleConstantsFactory ConstantCreator, int kFrontLeftDriveMotorId, int kFrontLeftSteerMotorId,
		int kFrontLeftEncoderId, Angle kFrontLeftEncoderOffset, boolean kFrontLeftSteerMotorInverted,
		boolean kFrontLeftCANcoderInverted, Distance kFrontLeftXPos, Distance kFrontLeftYPos,
		int kFrontRightDriveMotorId, int kFrontRightSteerMotorId, int kFrontRightEncoderId,
		Angle kFrontRightEncoderOffset, boolean kFrontRightSteerMotorInverted, boolean kFrontRightCANcoderInverted,
		Distance kFrontRightXPos, Distance kFrontRightYPos, int kBackLeftDriveMotorId, int kBackLeftSteerMotorId,
		int kBackLeftEncoderId, Angle kBackLeftEncoderOffset, boolean kBackLeftSteerMotorInverted,
		boolean kBackLeftCANcoderInverted, Distance kBackLeftXPos, Distance kBackLeftYPos, int kBackRightDriveMotorId,
		int kBackRightSteerMotorId, int kBackRightEncoderId, Angle kBackRightEncoderOffset,
		boolean kBackRightSteerMotorInverted, boolean kBackRightCANcoderInverted, Distance kBackRightXPos,
		Distance kBackRightYPos) {

	public SwerveModuleConstants getConstantsAtPos(int i)
	{
		return new SwerveModuleConstants();
	}
}
