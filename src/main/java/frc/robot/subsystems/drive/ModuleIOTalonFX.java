// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import frc.robot.util.DriveConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TunerConstants;

import java.util.Queue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ModuleIOTalonFX implements ModuleIO
{
	private final SwerveModuleConstants constants;
	private final TalonFX driveTalon;
	private final TalonFX turnTalon;
	private final CANcoder cancoder;

	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
	private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

	private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
	private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
	private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

	private final Queue<Double> timeStampQueue;

	private final StatusSignal<Angle> drivePosition;
	private final Queue<Double> drivePositionQueue;
	private final StatusSignal<AngularVelocity> driveVelocity;
	private final StatusSignal<Voltage> driveAppliedVolts;
	private final StatusSignal<Current> driveCurrent;

	private final StatusSignal<Angle> turnAbsolutePosition;
	private final Queue<Double> turnPositionQueue;
	private final StatusSignal<Angle> turnPosition;
	private final StatusSignal<AngularVelocity> turnVelocity;
	private final StatusSignal<Voltage> turnAppliedVolts;
	private final StatusSignal<Current> turnCurrent;
	private final TunerConstants tunerConstants;
	private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
	private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
	private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

	public ModuleIOTalonFX(SwerveModuleConstants constants, DriveConstants driveConstants,
			TunerConstants tunerConstants)
	{
		this.tunerConstants = tunerConstants;
		this.constants = constants;
		driveTalon = new TalonFX(constants.DriveMotorId, tunerConstants.DrivetrainConstants().CANBusName);
		turnTalon = new TalonFX(constants.DriveMotorId, tunerConstants.DrivetrainConstants().CANBusName);
		cancoder = new CANcoder(constants.CANcoderId, tunerConstants.DrivetrainConstants().CANBusName);
		var driveConfig = constants.DriveMotorInitialConfigs;
		driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		driveConfig.Slot0 = constants.DriveMotorGains;
		driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
		driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
		driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
		driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		PhoenixUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
		PhoenixUtil.tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

		var turnConfig = new TalonFXConfiguration();
		turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		turnConfig.Slot0 = constants.SteerMotorGains;
		turnConfig.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
		turnConfig.Feedback.FeedbackSensorSource = switch (constants.FeedbackSource)
		{
		case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
		case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
		case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
		};
		turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
		turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100 / constants.SteerMotorGearRatio;
		turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
		turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
		turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
		turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
		turnConfig.MotorOutput.Inverted = constants.SteerMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		PhoenixUtil.tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
		timeStampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

		drivePosition = driveTalon.getPosition();
		drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
		driveVelocity = driveTalon.getVelocity();
		driveAppliedVolts = driveTalon.getMotorVoltage();
		driveCurrent = driveTalon.getStatorCurrent();

		turnAbsolutePosition = cancoder.getAbsolutePosition();
		turnPosition = turnTalon.getPosition();
		turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());
		turnVelocity = turnTalon.getVelocity();
		turnAppliedVolts = turnTalon.getMotorVoltage();
		turnCurrent = turnTalon.getStatorCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(driveConstants.odometryFrequency(), drivePosition, turnPosition);
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent,
				turnAbsolutePosition, turnVelocity, turnAppliedVolts, turnCurrent);
		ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs)
	{
		var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
		var turnStatus = BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
		var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

		inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
		inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
		inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
		inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
		inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

		inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
		inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
		inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
		inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
		inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
		inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
		inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

		inputs.odometryTimestamps = timeStampQueue.stream().mapToDouble((Double value) -> value).toArray();
		inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
				.mapToDouble((Double value) -> Units.rotationsToRadians(value)).toArray();
		inputs.odometryTurnPositions = turnPositionQueue.stream().map((Double value) -> Rotation2d.fromRotations(value))
				.toArray(Rotation2d[]::new);
		timeStampQueue.clear();
		drivePositionQueue.clear();
		turnPositionQueue.clear();
	}

	@Override
	public void setDriveOpenLoop(double output)
	{
		driveTalon.setControl(switch (constants.DriveMotorClosedLoopOutput)
		{
		case Voltage -> voltageRequest.withOutput(output);
		case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
		});
	}

	@Override
	public void setTurnOpenLoop(double output)
	{
		turnTalon.setControl(switch (constants.SteerMotorClosedLoopOutput)
		{
		case Voltage -> voltageRequest.withOutput(output);
		case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
		});
	}

	@Override
	public void setDriveVelocity(double velocityRadPerSec)
	{
		double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
		driveTalon.setControl(switch (constants.DriveMotorClosedLoopOutput)
		{
		case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
		case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
		});
	}

	@Override
	public void setTurnPosition(Rotation2d rotation)
	{
		turnTalon.setControl(switch (constants.SteerMotorClosedLoopOutput)
		{
		case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
		case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(rotation.getRotations());
		});
	}
}
