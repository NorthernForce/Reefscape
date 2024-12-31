package frc.robot.zippy;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.robots.ZippyContainer;

public class ZippyDriverOI implements ZippyOI
{
	@Override
	public void bindDriverToXboxController(ZippyContainer container, CommandXboxController controller)
	{
	}

	@Override
	public void bindManipulatorToXboxController(ZippyContainer container, CommandXboxController controller)
	{
	}

	@Override
	public void bindDriverToJoystick(ZippyContainer container, CommandGenericHID joystick)
	{
	}

	@Override
	public void bindManipulatorToJoystick(ZippyContainer container, CommandGenericHID joystick)
	{
	}
}
