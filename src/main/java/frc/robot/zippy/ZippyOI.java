package frc.robot.zippy;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.robots.ZippyContainer;

public interface ZippyOI
{
	public void bindDriverToXboxController(ZippyContainer container, CommandXboxController controller);

	public void bindDriverToJoystick(ZippyContainer container, CommandGenericHID joystick);

	public void bindManipulatorToXboxController(ZippyContainer container, CommandXboxController controller);

	public void bindManipulatorToJoystick(ZippyContainer container, CommandGenericHID joystick);
}