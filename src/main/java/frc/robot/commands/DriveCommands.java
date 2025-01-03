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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

import com.pathplanner.lib.auto.AutoBuilder;

public class DriveCommands
{

	private DriveCommands()
	{
	}

	/**
	 * Field relative drive command using two joysticks (controlling linear and
	 * angular velocities).
	 *
	 * @param drive         the drive object instantiated with the motors.
	 * @param xSupplier     controller axis x of joystick
	 * @param ySupplier     controller axis y of joystick
	 * @param omegaSupplier speed of drive
	 * @return a runable command to drive el robo
	 */
	public static Command joystickDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
			DoubleSupplier omegaSupplier, double threshold)
	{
		return Commands.run(() ->
		{
			DoubleUnaryOperator process = v -> MathUtil.applyDeadband(v, threshold, 1.);
			double x = process.applyAsDouble(xSupplier.getAsDouble());
			double y = process.applyAsDouble(ySupplier.getAsDouble());
			double omega = process.applyAsDouble(omegaSupplier.getAsDouble());
			x *= Math.abs(x);
			y *= Math.abs(y);
			omega *= Math.abs(omega);
			ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);
			speeds.toRobotRelativeSpeeds(drive.getRotation());
			drive.runVelocity(speeds);
		}, drive);
	}

	/**
	 * Command to lock the robot's position and turns them inward, making it skid to
	 * a stop XD
	 *
	 * @param drive the drive object instantiated with the motors.
	 * @return a runable command to lock the robot's position
	 */
	public static Command xLock(Drive drive)
	{
		return Commands.run(() ->
		{
			// Lock the robot's position and turns them inward, making it skid to a stop XD
			drive.xLock();
		}, drive);
	}

	/**
	 * Command to reset the robot's orientation
	 *
	 * @param drive
	 * @return
	 */
	public static Command resetOrientaion(Drive drive)
	{
		return Commands.runOnce(() ->
		{
			drive.resetOrientation();
		}, drive);
	}

	/**
	 * Command to drive the robot to a pose on the field
	 */
	public static Command driveToPosition(Drive drive, Pose2d pose)
	{
        return Commands.runOnce(() -> AutoBuilder.followPath(drive.createPathToPose(pose)), drive);
	}
}
