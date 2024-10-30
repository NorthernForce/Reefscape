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

import org.littletonrobotics.junction.Logger;

public class Module
{
	private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
	static final double ODOMETRY_FREQUENCY = 250.0;

	private final ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private final int index;

	private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[]
	{};

	public Module(ModuleIO io, int index)
	{
		this.io = io;
		this.index = index;
		setBrakeMode(true);
	}

	/**
	 * Update inputs without running the rest of the periodic logic. This is useful
	 * since these updates need to be properly thread-locked.
	 */
	public void updateInputs()
	{
		io.updateInputs(inputs);
	}

	public void periodic()
	{
		Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

		// Calculate positions for odometry
		int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
		odometryPositions = new SwerveModulePosition[sampleCount];
		for (int i = 0; i < sampleCount; i++)
		{
			double positionMeters = inputs.odometryDrivePositionsMeters[i];
			Rotation2d angle = inputs.odometryTurnPositions[i];
			odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
		}
	}

	/**
	 * Runs the module with the specified setpoint state. Returns the optimized
	 * state.
	 */
	public SwerveModuleState runSetpoint(SwerveModuleState state)
	{
		// Optimize state based on current angle
		// Controllers run in "periodic" when the setpoint is not null
		var optimizedState = SwerveModuleState.optimize(state, getAngle());
		io.setDriveVelocity(optimizedState.speedMetersPerSecond);
		io.setTurnPosition(optimizedState.angle);
		return optimizedState;
	}

	/**
	 * Runs the module with the specified voltage while controlling to zero degrees.
	 */
	public void runCharacterization(double volts)
	{
		io.setTurnPosition(new Rotation2d());
		io.setDriveVoltage(volts);
	}

	/** Disables all outputs to motors. */
	public void stop()
	{
		io.setTurnVoltage(0.0);
		io.setDriveVoltage(0.0);
	}

	/** Sets whether brake mode is enabled. */
	public void setBrakeMode(boolean enabled)
	{
		io.setDriveBrakeMode(enabled);
		io.setTurnBrakeMode(enabled);
	}

	/** Returns the current turn angle of the module. */
	public Rotation2d getAngle()
	{
		return inputs.turnPosition;
	}

	/** Returns the current drive position of the module in meters. */
	public double getPositionMeters()
	{
		return inputs.drivePositionMeters;
	}

	/** Returns the current drive velocity of the module in meters per second. */
	public double getVelocityMetersPerSec()
	{
		return inputs.driveVelocityMetersPerSecond;
	}

	/** Returns the module position (turn angle and drive position). */
	public SwerveModulePosition getPosition()
	{
		return new SwerveModulePosition(getPositionMeters(), getAngle());
	}

	/** Returns the module state (turn angle and drive velocity). */
	public SwerveModuleState getState()
	{
		return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
	}

	/** Returns the module positions received this cycle. */
	public SwerveModulePosition[] getOdometryPositions()
	{
		return odometryPositions;
	}

	/** Returns the timestamps of the samples received this cycle. */
	public double[] getOdometryTimestamps()
	{
		return inputs.odometryTimestamps;
	}

	/** Returns the drive velocity in rad/sec. */
	public double getCharacterizationVelocity()
	{
		return inputs.driveVelocityMetersPerSecond / WHEEL_RADIUS;
	}
}
