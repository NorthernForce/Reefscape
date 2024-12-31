package frc.robot.zippy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.robots.ZippyContainer;

public class ZippyProgrammerOI implements ZippyOI
{
	@Override
	public void bindDriverToXboxController(ZippyContainer container, CommandXboxController controller)
	{
		controller.y().onTrue(
				DriveCommands.driveToPosition(container.drive, new Pose2d(new Translation2d(0, 0), new Rotation2d(0))));
	}

	@Override
	public void bindDriverToJoystick(ZippyContainer container, CommandGenericHID joystick)
	{
	}

	@Override
	public void bindManipulatorToXboxController(ZippyContainer container, CommandXboxController controller)
	{
	}

	@Override
	public void bindManipulatorToJoystick(ZippyContainer container, CommandGenericHID joystick)
	{
	}
}
