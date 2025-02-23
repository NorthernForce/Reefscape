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
            double x = MathUtil.applyDeadband(input.getAsDouble(), 0.0, 1);
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

        driverController.leftTrigger()
                .whileTrue(Commands.either(
                        container.getRollers().getAlgaeIntakeCommand(BlennyConstants.RollersConstants.INTAKE_SPEED),
                        container.getRollers().getCoralIntakeCommand(BlennyConstants.RollersConstants.INTAKE_SPEED),
                        () -> container.isInAlgaeState()));

        driverController.rightTrigger()
                .whileTrue(container.getRollers().getOuttakeCommand(BlennyConstants.RollersConstants.OUTTAKE_SPEED));

        driverController.a().whileTrue(container.getClimber().getClimbUpCommand(0.5));
        driverController.b().whileTrue(container.getClimber().getClimbDownCommand(0.5));

        driverController.start()
                .whileTrue(container.getSuperstructure().getHomingCommand(
                        BlennyConstants.InnerElevatorConstants.HOMING_SPEED,
                        BlennyConstants.OuterElevatorConstants.HOMING_SPEED));

        container.getRollers().setDefaultCommand(container.getRollers().getStopCommand());

        container.getSuperstructure().getWrist()
                .setDefaultCommand(container.getSuperstructure().getWrist().getStopCommand());

        manipulatorController.povLeft()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(BlennyConstants.SuperstructureGoal.L1));
        manipulatorController.povUp()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(BlennyConstants.SuperstructureGoal.L2));
        manipulatorController.povRight()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(BlennyConstants.SuperstructureGoal.L3));
        manipulatorController.povDown()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(BlennyConstants.SuperstructureGoal.L4));
        manipulatorController.a().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(BlennyConstants.SuperstructureGoal.CORAL_STATION));
        manipulatorController.b().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(BlennyConstants.SuperstructureGoal.PROCESSOR_STATION));
        manipulatorController.y().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(BlennyConstants.SuperstructureGoal.LOWER_ALGAE));
        manipulatorController.x().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(BlennyConstants.SuperstructureGoal.HIGHER_ALGAE));

        manipulatorController.leftBumper().whileTrue(container.getSuperstructure().getWrist()
                .getSetSpeedCommand(-BlennyConstants.WristJointConstants.MANUAL_MOVE_SPEED));

        manipulatorController.rightBumper().whileTrue(container.getSuperstructure().getWrist()
                .getSetSpeedCommand(BlennyConstants.WristJointConstants.MANUAL_MOVE_SPEED));

        manipulatorController.leftTrigger()
                .whileTrue(Commands.either(
                        container.getRollers().getAlgaeIntakeCommand(BlennyConstants.RollersConstants.INTAKE_SPEED),
                        container.getRollers().getCoralIntakeCommand(BlennyConstants.RollersConstants.INTAKE_SPEED),
                        () -> container.isInAlgaeState()));

        manipulatorController.rightTrigger()
                .whileTrue(container.getRollers().getOuttakeCommand(BlennyConstants.RollersConstants.OUTTAKE_SPEED));

        manipulatorController.start()
                .whileTrue(container.getSuperstructure().getHomingCommand(
                        BlennyConstants.InnerElevatorConstants.HOMING_SPEED,
                        BlennyConstants.OuterElevatorConstants.HOMING_SPEED));

        container.getSuperstructure().getInnerElevator().setDefaultCommand(container.getSuperstructure()
                .getInnerElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getRightY)));

        container.getSuperstructure().getOuterElevator().setDefaultCommand(container.getSuperstructure()
                .getOuterElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getLeftY)));
    }
}
