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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and CANcoder
 *
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware configurations (e.g. If using an analog encoder, copy from
 * "ModuleIOSparkMax")
 *
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward motion on the drive motor will propel the robot forward) and
 * copy the reported values from the absolute encoders using AdvantageScope.
 * These values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO
{
	private final TalonFX driveTalon;
	private final TalonFX turnTalon;
	private final CANcoder cancoder;

	private final StatusSignal<Angle> drivePosition;
	private final StatusSignal<AngularVelocity> driveVelocity;
	private final StatusSignal<Voltage> driveAppliedVolts;
	private final StatusSignal<Current> driveCurrent;

	private final StatusSignal<Angle> turnPosition;
	private final StatusSignal<AngularVelocity> turnVelocity;
	private final StatusSignal<Voltage> turnAppliedVolts;
	private final StatusSignal<Current> turnCurrent;
	private final StatusSignal<Temperature> driveTemperature;
	private final StatusSignal<Temperature> turnTemperature;

	private final double driveGearRatio;
	private final double wheelCircumference;

	public ModuleIOTalonFX(int driveId, int turnId, int encoderId, double turnGearRatio, double driveGearRatio,
			boolean invertDriveMotor, boolean invertTurnMotor, double driveCurrentLimit, double turnCurrentLimit,
			double odometryFrequency, double wheelRadius, double driveP, double driveI, double driveV, double turnP,
			double turnD)
	{
		this.driveGearRatio = driveGearRatio;
		this.wheelCircumference = wheelRadius * 2 * Math.PI;
		driveTalon = new TalonFX(driveId);
		turnTalon = new TalonFX(turnId);
		cancoder = new CANcoder(encoderId);

		var driveConfig = new TalonFXConfiguration();
		driveConfig.CurrentLimits.SupplyCurrentLimit = driveCurrentLimit;
		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveConfig.CurrentLimits.StatorCurrentLimit = driveCurrentLimit;
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.Slot0.kP = driveP;
		driveConfig.Slot0.kI = driveI;
		driveConfig.Slot0.kV = driveV;
		driveTalon.getConfigurator().apply(driveConfig);
		driveTalon.setInverted(invertDriveMotor);
		setDriveBrakeMode(true);

		var turnConfig = new TalonFXConfiguration();
		turnConfig.CurrentLimits.SupplyCurrentLimit = turnCurrentLimit;
		turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		turnConfig.CurrentLimits.StatorCurrentLimit = turnCurrentLimit;
		turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		turnConfig.Feedback.FeedbackRemoteSensorID = encoderId;
		turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
		turnConfig.Feedback.RotorToSensorRatio = turnGearRatio;
		turnConfig.Feedback.SensorToMechanismRatio = 1.0;
		turnConfig.Slot0.kP = turnP;
		turnConfig.Slot0.kD = turnD;
		turnTalon.getConfigurator().apply(turnConfig);
		turnTalon.setInverted(invertTurnMotor);
		setTurnBrakeMode(true);

		drivePosition = driveTalon.getPosition();
		driveVelocity = driveTalon.getVelocity();
		driveAppliedVolts = driveTalon.getMotorVoltage();
		driveCurrent = driveTalon.getSupplyCurrent();

		turnPosition = turnTalon.getPosition();
		turnVelocity = turnTalon.getVelocity();
		turnAppliedVolts = turnTalon.getMotorVoltage();
		turnCurrent = turnTalon.getSupplyCurrent();

		driveTemperature = driveTalon.getDeviceTemp();
		turnTemperature = turnTalon.getDeviceTemp();

		BaseStatusSignal.setUpdateFrequencyForAll(odometryFrequency, drivePosition, turnPosition);
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent, turnVelocity,
				turnAppliedVolts, turnCurrent, driveTemperature, turnTemperature);
		driveTalon.optimizeBusUtilization();
		turnTalon.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs)
	{
		BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent, turnPosition,
				turnVelocity, turnAppliedVolts, turnCurrent, driveTemperature, turnTemperature);

		inputs.drivePositionMeters = drivePosition.getValueAsDouble() / driveGearRatio * wheelCircumference;
		inputs.driveVelocityMetersPerSecond = driveVelocity.getValueAsDouble() / driveGearRatio * wheelCircumference;
		inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
		inputs.driveCurrentAmps = new double[]
		{ driveCurrent.getValueAsDouble() };

		inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
		inputs.turnVelocityRotationsPerSecond = turnVelocity.getValueAsDouble();
		inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
		inputs.turnCurrentAmps = new double[]
		{ turnCurrent.getValueAsDouble() };

		inputs.driveTemperature = driveTemperature.getValueAsDouble();
		inputs.turnTemperature = turnTemperature.getValueAsDouble();
	}

	@Override
	public void setDriveVoltage(double volts)
	{
		driveTalon.setControl(new VoltageOut(volts));
	}

	@Override
	public void setTurnVoltage(double volts)
	{
		turnTalon.setControl(new VoltageOut(volts));
	}

	@Override
	public void setDriveVelocity(double velocityMetersPerSecond)
	{
		driveTalon.setControl(
				new VelocityTorqueCurrentFOC(velocityMetersPerSecond / wheelCircumference * driveGearRatio));
	}

	@Override
	public void setTurnPosition(Rotation2d position)
	{
		turnTalon.setControl(new PositionVoltage(position.getRotations()));
	}

	@Override
	public void setDriveBrakeMode(boolean enable)
	{
		var config = new MotorOutputConfigs();
		driveTalon.getConfigurator().refresh(config);
		config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		driveTalon.getConfigurator().apply(config);
	}

	@Override
	public void setTurnBrakeMode(boolean enable)
	{
		var config = new MotorOutputConfigs();
		turnTalon.getConfigurator().refresh(config);
		config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		turnTalon.getConfigurator().apply(config);
	}
}
