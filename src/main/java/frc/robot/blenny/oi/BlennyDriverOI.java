package frc.robot.blenny.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.blenny.BlennyContainer;
import frc.robot.blenny.constants.BlennyConstants;

/**
 * Blenny OI for the driver and operator
 */
public class BlennyDriverOI implements BlennyOI
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
    public void bindOI(BlennyContainer container)
    {
        CommandXboxController driverController = new CommandXboxController(0);

        container.getDrive().setDefaultCommand(container.getDrive().getDriveByJoystickCommand(
                processJoystickInput(driverController::getLeftY), processJoystickInput(driverController::getLeftX),
                processJoystickInput(driverController::getRightX)));

        driverController.back().onTrue(container.getDrive()
                .getResetOrientationCommand(FieldConstants.getFieldRotation(FieldConstants.getAlliance())));

        driverController.x().whileTrue(container.getDrive().getXLockCommand());

        driverController.leftTrigger().whileTrue(Commands.either(
            container.getRollers().getAlgaeIntakeCommand(BlennyConstants.RollersConstants.INTAKE_SPEED),
            container.getRollers().getCoralIntakeCommand(BlennyConstants.RollersConstants.INTAKE_SPEED),
            () -> container.isInAlgaeState()));
        
        driverController.rightTrigger().whileTrue(container.getRollers().getOuttakeCommand(1));

        driverController.leftBumper().whileTrue(container.getClimber().getClimbUpCommand(0.5));
        driverController.rightBumper().whileTrue(container.getClimber().getClimbDownCommand(0.5));

        container.getRollers().setDefaultCommand(container.getRollers().getStopCommand());

    }
}
