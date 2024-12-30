package frc.robot.zippy.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import frc.robot.util.DriveConstants;
import frc.robot.util.TunerConstants;

// EVERYTHING THAT MUST BE SET IN THE TUNER CONSTANTS:

// private Slot0Configs steerGains = new
// Slot0Configs().withKP(100).withKI(0).withKD(2.0).withKS(0.2)
// .withKV(1.59).withKA(0).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
// private Slot0Configs driveGains = new
// Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0)
// .withKV(0.124);

// private ClosedLoopOutputType kSteerClosedLoopOutput =
// ClosedLoopOutputType.Voltage;
// private ClosedLoopOutputType kDriveClosedLoopOutput =
// ClosedLoopOutputType.Voltage;

// private SteerFeedbackType kSteerFeedbackType =
// SteerFeedbackType.FusedCANcoder;

// private Current kSlipCurrent = Amps.of(120.0);

// private TalonFXConfiguration driveInitialConfigs = new
// TalonFXConfiguration();
// private TalonFXConfiguration steerInitialConfigs = new
// TalonFXConfiguration().withCurrentLimits(
// new
// CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(60)).withStatorCurrentLimitEnable(true));
// private CANcoderConfiguration cancoderInitialConfigs = new
// CANcoderConfiguration();
// private Pigeon2Configuration pigeonConfigs = null;

// public CANBus kCANBus = new CANBus("rio", "./logs/example.hoot");

// public LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.55);

// private double kCoupleRatio = 3.5;

// private double kDriveGearRatio = 7.363636364;
// private double kSteerGearRatio = 12.8;
// private Distance kWheelRadius = Inches.of(2.167);

// private boolean kInvertLeftSide = false;
// private boolean kInvertRightSide = true;

// private int kPigeonId = 1;

// private double kSteerInertia = 0.004;
// private double kDriveInertia = 0.025;
// private Voltage kSteerFrictionVoltage = Volts.of(0.25);
// private Voltage kDriveFrictionVoltage = Volts.of(0.25);

// public SwerveDrivetrainConstants DrivetrainConstants = new
// SwerveDrivetrainConstants()
// .withCANBusName(kCANBus.getName()).withPigeon2Id(kPigeonId).withPigeon2Configs(pigeonConfigs);

// private SwerveModuleConstantsFactory ConstantCreator = new
// SwerveModuleConstantsFactory()
// .withDriveMotorGearRatio(kDriveGearRatio).withSteerMotorGearRatio(kSteerGearRatio)
// .withCouplingGearRatio(kCoupleRatio).withWheelRadius(kWheelRadius).withSteerMotorGains(steerGains)
// .withDriveMotorGains(driveGains).withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
// .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput).withSlipCurrent(kSlipCurrent)
// .withSpeedAt12Volts(kSpeedAt12Volts).withFeedbackSource(kSteerFeedbackType)
// .withDriveMotorInitialConfigs(driveInitialConfigs).withSteerMotorInitialConfigs(steerInitialConfigs)
// .withCANcoderInitialConfigs(cancoderInitialConfigs).withSteerInertia(kSteerInertia)
// .withDriveInertia(kDriveInertia).withSteerFrictionVoltage(kSteerFrictionVoltage)
// .withDriveFrictionVoltage(kDriveFrictionVoltage);

// private int kFrontLeftDriveMotorId = 5;
// private int kFrontLeftSteerMotorId = 4;
// private int kFrontLeftEncoderId = 2;
// private Angle kFrontLeftEncoderOffset = Rotations.of(-0.83544921875);
// private boolean kFrontLeftSteerMotorInverted = true;
// private boolean kFrontLeftCANcoderInverted = false;

// private Distance kFrontLeftXPos = Inches.of(10.5);
// private Distance kFrontLeftYPos = Inches.of(10.5);

// private int kFrontRightDriveMotorId = 7;
// private int kFrontRightSteerMotorId = 6;
// private int kFrontRightEncoderId = 3;
// private Angle kFrontRightEncoderOffset = Rotations.of(-0.15234375);
// private boolean kFrontRightSteerMotorInverted = true;
// private boolean kFrontRightCANcoderInverted = false;

// private Distance kFrontRightXPos = Inches.of(10.5);
// private Distance kFrontRightYPos = Inches.of(-10.5);

// private int kBackLeftDriveMotorId = 1;
// private int kBackLeftSteerMotorId = 0;
// private int kBackLeftEncoderId = 0;
// private Angle kBackLeftEncoderOffset = Rotations.of(-0.4794921875);
// private boolean kBackLeftSteerMotorInverted = true;
// private boolean kBackLeftCANcoderInverted = false;

// private Distance kBackLeftXPos = Inches.of(-10.5);
// private Distance kBackLeftYPos = Inches.of(10.5);

// private int kBackRightDriveMotorId = 3;
// private int kBackRightSteerMotorId = 2;
// private int kBackRightEncoderId = 1;
// private Angle kBackRightEncoderOffset = Rotations.of(-0.84130859375);
// private boolean kBackRightSteerMotorInverted = true;
// private boolean kBackRightCANcoderInverted = false;

// private Distance kBackRightXPos = Inches.of(-10.5);
// private Distance kBackRightYPos = Inches.of(-10.5);

// public SwerveModuleConstants FrontLeft =
// ConstantCreator.createModuleConstants(
// kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
// kFrontLeftEncoderOffset,
// kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide,
// kFrontLeftSteerMotorInverted,
// kFrontLeftCANcoderInverted);
// public SwerveModuleConstants FrontRight =
// ConstantCreator.createModuleConstants(
// kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
// kFrontRightEncoderOffset,
// kFrontRightXPos, kFrontRightYPos, kInvertRightSide,
// kFrontRightSteerMotorInverted,
// kFrontRightCANcoderInverted);
// public SwerveModuleConstants BackLeft =
// ConstantCreator.createModuleConstants(
// kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
// kBackLeftEncoderOffset, kBackLeftXPos,
// kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted,
// kBackLeftCANcoderInverted);
// public SwerveModuleConstants BackRight =
// ConstantCreator.createModuleConstants(
// kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
// kBackRightEncoderOffset,
// kBackRightXPos, kBackRightYPos, kInvertRightSide,
// kBackRightSteerMotorInverted,
// kBackRightCANcoderInverted);

// public static SwerveModuleConstants getConstantsAtPos(int position)
// {
// return new SwerveModuleConstants(); // TODO: talk to connnor about slick
// phoenix generation stuff
// }

public class ZippyConstants
{
	public static TunerConstants tunerConstants = new TunerConstants(
			new Slot0Configs().withKP(100).withKI(0).withKD(2.0).withKS(0.2).withKV(1.59).withKA(0)
					.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign),
			new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124), ClosedLoopOutputType.Voltage,
			ClosedLoopOutputType.Voltage, SteerFeedbackType.FusedCANcoder, Amps.of(120.0), new TalonFXConfiguration(),
			new TalonFXConfiguration().withCurrentLimits(
					new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(60)).withStatorCurrentLimitEnable(true)),
			new CANcoderConfiguration(), null, new CANBus("rio", "./logs/example.hoot"), MetersPerSecond.of(4.55), 3.5,
			7.363636364, 12.8, Inches.of(2.167), false, true, 1, 0.004, 0.025, Volts.of(0.25), Volts.of(0.25),
			new SwerveDrivetrainConstants().withCANBusName("rio").withPigeon2Id(1).withPigeon2Configs(null),
			new SwerveModuleConstantsFactory().withDriveMotorGearRatio(7.363636364).withSteerMotorGearRatio(12.8)
					.withCouplingGearRatio(3.5).withWheelRadius(Inches.of(2.167))
					.withSteerMotorGains(new Slot0Configs().withKP(100).withKI(0).withKD(2.0).withKS(0.2).withKV(1.59)
							.withKA(0).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
					.withDriveMotorGains(new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124))
					.withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
					.withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage).withSlipCurrent(Amps.of(120.0))
					.withSpeedAt12Volts(MetersPerSecond.of(4.55)).withFeedbackSource(SteerFeedbackType.FusedCANcoder)
					.withDriveMotorInitialConfigs(new TalonFXConfiguration())
					.withSteerMotorInitialConfigs(
							new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
									.withStatorCurrentLimit(Amps.of(60)).withStatorCurrentLimitEnable(true)))
					.withCANcoderInitialConfigs(new CANcoderConfiguration()).withSteerInertia(0.004)
					.withDriveInertia(0.025).withSteerFrictionVoltage(Volts.of(0.25))
					.withDriveFrictionVoltage(Volts.of(0.25)),
			5, 4, 2, Rotations.of(-0.83544921875), true, false, Inches.of(10.5), Inches.of(10.5), 7, 6, 3,
			Rotations.of(-0.15234375), true, false, Inches.of(10.5), Inches.of(-10.5), 1, 0, 0,
			Rotations.of(-0.4794921875), true, false, Inches.of(-10.5), Inches.of(10.5), 3, 2, 1,
			Rotations.of(-0.84130859375), true, false, Inches.of(-10.5), Inches.of(-10.5));
	public static DriveConstants driveConstants = new DriveConstants(50, 3, 0.5, 0.5, 0.5, 3);
}
