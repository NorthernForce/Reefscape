package frc.robot.blenny.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.blenny.BlennyContainer;
import frc.robot.blenny.constants.BlennyConstants;

/**
 * Blenny OI for the programmers
 */
public class BlennyProgrammerOI implements BlennyOI
{
    private static DoubleSupplier processJoystickInput(DoubleSupplier input)
    {
        return () ->
        {
            double x = MathUtil.applyDeadband(input.getAsDouble(), 0.1, 1);
            return -x * Math.abs(x);
        };
    }

    @Override
    public void bindOI(BlennyContainer container)
    {
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController manipulatorController = new CommandXboxController(1);

        container.getDrive().setDefaultCommand(container.getDrive().getDriveByJoystickCommand(
                processJoystickInput(driverController::getLeftY), processJoystickInput(driverController::getLeftX),
                processJoystickInput(driverController::getRightX)));

        driverController.back().onTrue(container.getDrive()
                .getResetOrientationCommand(FieldConstants.getFieldRotation(FieldConstants.getAlliance())));

        driverController.x().whileTrue(container.getDrive().getXLockCommand());

        container.getSuperstructure().getWrist()
                .setDefaultCommand(container.getSuperstructure().getWrist().getStopCommand());

        manipulatorController.leftBumper().whileTrue(container.getSuperstructure().getWrist()
                .getSetSpeedCommand(-BlennyConstants.WristJointConstants.MANUAL_MOVE_SPEED));

        manipulatorController.rightBumper().whileTrue(container.getSuperstructure().getWrist()
                .getSetSpeedCommand(BlennyConstants.WristJointConstants.MANUAL_MOVE_SPEED));

        driverController.b().whileTrue(container.getLEDs().getSetColour(0, 255, 0));
        driverController.y().whileTrue(container.getLEDs().getIncrementAnimation());
    }

}
