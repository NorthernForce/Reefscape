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

package frc.robot.robots;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import java.util.Map;

import org.northernforce.util.NFRRobotContainer;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class CrabbyContainer implements NFRRobotContainer
{
	// Subsystems
	private final Drive drive;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public CrabbyContainer()
	{
		switch (Constants.kCurrentMode)
		{
		case REAL:
			// Real robot, instantiate hardware IO implementations
			drive = new Drive(new GyroIOPigeon2(false), CrabbyConstants.DriveConstants.MAX_LINEAR_SPEED,
					CrabbyConstants.DriveConstants.MAX_ANGULAR_SPEED, CrabbyConstants.DriveConstants.DRIVE_BASE_RADIUS,
					CrabbyConstants.DriveConstants.TRACK_WIDTH_X, CrabbyConstants.DriveConstants.TRACK_WIDTH_Y,
					Constants.kCurrentMode, CrabbyConstants.DriveConstants.driveFeedforwardks,
					CrabbyConstants.DriveConstants.driveFeedforwardkv, CrabbyConstants.DriveConstants.driveFeedbackkp,
					CrabbyConstants.DriveConstants.driveFeedbackki, CrabbyConstants.DriveConstants.driveFeedbackkd,
					CrabbyConstants.DriveConstants.turnFeedbackkp, CrabbyConstants.DriveConstants.turnFeedbackki,
					CrabbyConstants.DriveConstants.turnFeedbackkd,
					new ModuleIOTalonFX(0, 1, 2, CrabbyConstants.DriveConstants.TURN_GEAR_RATIO,
							CrabbyConstants.DriveConstants.DRIVE_GEAR_RATIO),
					new ModuleIOTalonFX(1, 2, 3, CrabbyConstants.DriveConstants.TURN_GEAR_RATIO,
							CrabbyConstants.DriveConstants.DRIVE_GEAR_RATIO),
					new ModuleIOTalonFX(2, 3, 4, CrabbyConstants.DriveConstants.TURN_GEAR_RATIO,
							CrabbyConstants.DriveConstants.DRIVE_GEAR_RATIO),
					new ModuleIOTalonFX(3, 4, 5, CrabbyConstants.DriveConstants.TURN_GEAR_RATIO,
							CrabbyConstants.DriveConstants.DRIVE_GEAR_RATIO));
			// drive = new Drive(
			// new GyroIOPigeon2(true),
			// new ModuleIOTalonFX(0),
			// new ModuleIOTalonFX(1),
			// new ModuleIOTalonFX(2),
			// new ModuleIOTalonFX(3));
			// flywheel = new Flywheel(new FlywheelIOTalonFX());
			break;

		case SIM:
			// Sim robot, instantiate physics sim IO implementations
			drive = new Drive(new GyroIO()
			{
			}, CrabbyConstants.DriveConstants.MAX_LINEAR_SPEED, CrabbyConstants.DriveConstants.MAX_ANGULAR_SPEED,
					CrabbyConstants.DriveConstants.DRIVE_BASE_RADIUS, CrabbyConstants.DriveConstants.TRACK_WIDTH_X,
					CrabbyConstants.DriveConstants.TRACK_WIDTH_Y, Constants.kCurrentMode,
					CrabbyConstants.DriveConstants.driveFeedforwardks,
					CrabbyConstants.DriveConstants.driveFeedforwardkv, CrabbyConstants.DriveConstants.driveFeedbackkp,
					CrabbyConstants.DriveConstants.driveFeedbackki, CrabbyConstants.DriveConstants.driveFeedbackkd,
					CrabbyConstants.DriveConstants.turnFeedbackkp, CrabbyConstants.DriveConstants.turnFeedbackki,
					CrabbyConstants.DriveConstants.turnFeedbackkd, new ModuleIOSim(), new ModuleIOSim(),
					new ModuleIOSim(), new ModuleIOSim());
			break;

		default:
			// Replayed robot, disable IO implementations
			drive = new Drive(new GyroIO()
			{
			}, CrabbyConstants.DriveConstants.MAX_LINEAR_SPEED, CrabbyConstants.DriveConstants.MAX_ANGULAR_SPEED,
					CrabbyConstants.DriveConstants.DRIVE_BASE_RADIUS, CrabbyConstants.DriveConstants.TRACK_WIDTH_X,
					CrabbyConstants.DriveConstants.TRACK_WIDTH_Y, Constants.kCurrentMode,
					CrabbyConstants.DriveConstants.driveFeedforwardks,
					CrabbyConstants.DriveConstants.driveFeedforwardkv, CrabbyConstants.DriveConstants.driveFeedbackkp,
					CrabbyConstants.DriveConstants.driveFeedbackki, CrabbyConstants.DriveConstants.driveFeedbackkd,
					CrabbyConstants.DriveConstants.turnFeedbackkp, CrabbyConstants.DriveConstants.turnFeedbackki,
					CrabbyConstants.DriveConstants.turnFeedbackkd, new ModuleIO()
					{
					}, new ModuleIO()
					{
					}, new ModuleIO()
					{
					}, new ModuleIO()
					{
					});
		{
		}
			;
			break;
		}

	}

	public Map<String, Command> getAutonomousOptions()
	{
		return Map.of();
	}

	public Map<String, Pose2d> getStartingLocations()
	{
		return Map.of();
	}

	public Pair<String, Command> getDefaultAutonomous()
	{
		return Pair.of("nothing", Commands.none());
	}

	public void setInitialPose(Pose2d pose)
	{

	}

	public void bindOI(GenericHID driverHID, GenericHID manipulatorHID)
	{

	}
}
