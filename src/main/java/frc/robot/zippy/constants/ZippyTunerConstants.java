package frc.robot.zippy.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.measure.*;

public class ZippyTunerConstants
{
	// The steer motor uses any SwerveModule.SteerRequestType control request with
	// the
	// output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
	public static final Slot0Configs steerGains = new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0)
			.withKV(1.5).withKA(0);
	// When using closed-loop control, the drive motor uses the control
	// output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
	public static final Slot0Configs driveGains = new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0)
			.withKA(0);

	private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
	private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

	private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

	private static final Current kSlipCurrent = Amps.of(120.0);

	private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
	private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration().withCurrentLimits(
			new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(60)).withStatorCurrentLimitEnable(true));
	private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
	private static final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

	public static final CANBus kCANBus = new CANBus("drive");

	public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.21);

	private static final double kCoupleRatio = 3.5714285714285716;

	private static final double kDriveGearRatio = 6.122448979591837;
	private static final double kSteerGearRatio = 12.8;
	private static final Distance kWheelRadius = Inches.of(2);

	private static final boolean kInvertLeftSide = true;
	private static final boolean kInvertRightSide = false;

	private static final int kPigeonId = 1;

	private static final double kSteerInertia = 0.00001;
	private static final double kDriveInertia = 0.001;
	private static final Voltage kSteerFrictionVoltage = Volts.of(0.25);
	private static final Voltage kDriveFrictionVoltage = Volts.of(0.25);

	public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
			.withCANBusName(kCANBus.getName()).withPigeon2Id(kPigeonId).withPigeon2Configs(pigeonConfigs);

	private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
			.withDriveMotorGearRatio(kDriveGearRatio).withSteerMotorGearRatio(kSteerGearRatio)
			.withCouplingGearRatio(kCoupleRatio).withWheelRadius(kWheelRadius).withSteerMotorGains(steerGains)
			.withDriveMotorGains(driveGains).withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
			.withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput).withSlipCurrent(kSlipCurrent)
			.withSpeedAt12Volts(kSpeedAt12Volts).withFeedbackSource(kSteerFeedbackType)
			.withDriveMotorInitialConfigs(driveInitialConfigs).withSteerMotorInitialConfigs(steerInitialConfigs)
			.withCANcoderInitialConfigs(cancoderInitialConfigs).withSteerInertia(kSteerInertia)
			.withDriveInertia(kDriveInertia).withSteerFrictionVoltage(kSteerFrictionVoltage)
			.withDriveFrictionVoltage(kDriveFrictionVoltage);

	public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(2, 1, 1,
			Rotations.of(-0.177490234375), Inches.of(11.125), Inches.of(11.125), kInvertLeftSide, false, false);

	public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(4, 3, 2,
			Rotations.of(0.052001953125), Inches.of(11.125), Inches.of(-11.125), kInvertRightSide, false, false);

	public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(6, 5, 3,
			Rotations.of(-0.423828125), Inches.of(-11.125), Inches.of(11.125), kInvertLeftSide, false, false);

	public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(8, 7, 4,
			Rotations.of(-0.291748046875), Inches.of(-11.125), Inches.of(-11.125), kInvertRightSide, false, false);
}