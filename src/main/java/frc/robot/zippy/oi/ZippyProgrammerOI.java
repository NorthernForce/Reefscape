package frc.robot.zippy.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.zippy.ZippyContainer;

public class ZippyProgrammerOI implements ZippyOI
{
	/**
	 * Process joystick input (meant for XBoxController)
	 * 
	 * @param input the input to process
	 * @return the processed input (squared and deadbanded)
	 */
	private static DoubleSupplier processJoystickInput(DoubleSupplier input)
	{
		return () ->
		{
			double x = MathUtil.applyDeadband(input.getAsDouble(), 0.1, 1);
			return -x * Math.abs(x);
		};
	}

	@Override
	public void bindOI(ZippyContainer container)
	{
		CommandXboxController driverJoystick = new CommandXboxController(0);

		container.getDrive()
				.setDefaultCommand(container.getDrive().getDriveByJoystickCommand(
						processJoystickInput(driverJoystick::getLeftY), processJoystickInput(driverJoystick::getLeftX),
						processJoystickInput(driverJoystick::getRightX)));

		driverJoystick.x().whileTrue(container.getDrive().getXLockCommand());
	}
}
