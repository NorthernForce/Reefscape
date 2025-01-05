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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public class Module
{
	private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
	static final double ODOMETRY_FREQUENCY = 250.0;
	private final ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private final int index;
	private final SwerveModuleConstants constants;

	private final Alert driveDisconnectedAlert;
	private final Alert turnDisconnectedAlert;
	private final Alert turnEncoderDisconnectedAlert;
	private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[]
	{};

	public void updateInputs()
	{
		io.updateInputs(inputs);
	}

	public void setBrakeMode(boolean enabled)
	{
		io.setDriveBrakeMode(enabled);
		io.setTurnBrakeMode(enabled);
	}

	public double getCharacterizationVelocity()
	{
		return inputs.driveVelocityMetersPerSecond / WHEEL_RADIUS;
	}

	public Module(ModuleIO io, int index, SwerveModuleConstants constants)
	{
		this.io = io;
		this.index = index;
		this.constants = constants;
		driveDisconnectedAlert = new Alert("Disconnected drive motor on module " + Integer.toString(index) + ".",
				AlertType.kError);
		turnDisconnectedAlert = new Alert("Disconnected turn motor on module " + Integer.toString(index) + ".",
				AlertType.kError);
		turnEncoderDisconnectedAlert = new Alert("Disconnected turn encoder on module " + Integer.toString(index) + ".",
				AlertType.kError);
	}

	public void periodic()
	{
		io.updateInputs(inputs);
		Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

		int sampleCount = inputs.odometryTimestamps.length;
		odometryPositions = new SwerveModulePosition[sampleCount];
		for (int i = 0; i < sampleCount; i++)
		{
			double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
			Rotation2d angle = inputs.odometryTurnPositions[i];
			odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
		}

		driveDisconnectedAlert.set(!inputs.driveConnected);
		turnDisconnectedAlert.set(!inputs.turnConnected);
		turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
	}

	public void runSetpoint(SwerveModuleState state)
	{
		state.optimize(getAngle());
		state.cosineScale(inputs.turnPosition);

		io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
		io.setTurnPosition(state.angle);
	}

	public void runCharacterization(double output)
	{
		io.setDriveOpenLoop(output);
		io.setTurnPosition(Rotation2d.kZero);
	}

	public void stop()
	{
		io.setDriveOpenLoop(0.0);
		io.setTurnOpenLoop(0.0);
	}

	public Rotation2d getAngle()
	{
		return inputs.turnPosition;
	}

	public double getPositionMeters()
	{
		return inputs.drivePositionRad * constants.WheelRadius;
	}

	public double getVelocityMetersPerSec()
	{
		return inputs.driveVelocityRadPerSec * constants.WheelRadius;
	}

	public SwerveModulePosition getPosition()
	{
		return new SwerveModulePosition(getPositionMeters(), getAngle());
	}

	public SwerveModuleState getState()
	{
		return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
	}

	public SwerveModulePosition[] getOdometryPositions()
	{
		return odometryPositions;
	}

	public double[] getOdometryTimestamps()
	{
		return inputs.odometryTimestamps;
	}

	public double getWheelRadiusCharacterizationPosition()
	{
		return inputs.drivePositionRad;
	}

	public double getFFCharacterizationVelocity()
	{
		return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
	}
}
