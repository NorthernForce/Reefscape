package frc.robot.crabby;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.robots.CrabbyContainer;

/**
 * An OI for the programmers when operating Crabby. Features more programming
 * features not needed by drivers.
 */
public class CrabbyProgrammerOI implements CrabbyOI
{
	@Override
	public void bindOI(CrabbyContainer container)
	{
		final CommandXboxController driverController = new CommandXboxController(0);
		final CommandXboxController manipulatorController = new CommandXboxController(0);
		container.drive.setDefaultCommand(DriveCommands.joystickDrive(container.drive,
				() -> driverController.getLeftX(), () -> driverController.getLeftY(),
				() -> Math.atan2(driverController.getRightY(), driverController.getRightX())));
		driverController.x().onTrue(DriveCommands.xLock(container.drive));
		driverController.y().onTrue(DriveCommands.resetOrientaion(container.drive));
	}

}
