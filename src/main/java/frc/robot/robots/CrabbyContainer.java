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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class CrabbyContainer
{
	// Subsystems
	private final Drive drive;

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public CrabbyContainer()
	{
		switch (Constants.kCurrentMode)
		{
		case REAL:
			// Real robot, instantiate hardware IO implementations
			drive = new Drive(new GyroIOPigeon2(false), Constants.MAX_LINEAR_SPEED, Constants.MAX_ANGULAR_SPEED,
					Constants.DRIVE_BASE_RADIUS, Constants.TRACK_WIDTH_X, Constants.TRACK_WIDTH_Y,
					Constants.kCurrentMode,
					new ModuleIOSparkMax(0, 1, 2, Constants.TURN_GEAR_RATIO, Constants.DRIVE_GEAR_RATIO),
					new ModuleIOSparkMax(1, 2, 3, Constants.TURN_GEAR_RATIO, Constants.DRIVE_GEAR_RATIO),
					new ModuleIOSparkMax(2, 3, 4, Constants.TURN_GEAR_RATIO, Constants.DRIVE_GEAR_RATIO),
					new ModuleIOSparkMax(3, 4, 5, Constants.TURN_GEAR_RATIO, Constants.DRIVE_GEAR_RATIO));
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
			}, Constants.MAX_LINEAR_SPEED, Constants.MAX_ANGULAR_SPEED, Constants.DRIVE_BASE_RADIUS,
					Constants.TRACK_WIDTH_X, Constants.TRACK_WIDTH_Y, Constants.kCurrentMode, new ModuleIOSim(),
					new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
			break;

		default:
			// Replayed robot, disable IO implementations
			drive = new Drive(new GyroIO()
			{
			}, Constants.MAX_LINEAR_SPEED, Constants.MAX_ANGULAR_SPEED, Constants.DRIVE_BASE_RADIUS,
					Constants.TRACK_WIDTH_X, Constants.TRACK_WIDTH_Y, Constants.kCurrentMode, new ModuleIO()
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

		// Set up auto routines
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		// Set up SysId routines
		autoChooser.addOption("Drive SysId (Quasistatic Forward)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand()
	{
		return autoChooser.get();
	}
}
