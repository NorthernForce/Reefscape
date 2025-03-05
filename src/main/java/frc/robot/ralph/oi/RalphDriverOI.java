package frc.robot.ralph.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.commands.RumbleXBoxController;
import frc.robot.ralph.RalphContainer;
import frc.robot.ralph.constants.RalphConstants;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;

/**
 * Ralph OI for the driver and operator
 */
public class RalphDriverOI implements RalphOI
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

    static void bindDrive(CommandXboxController driverController, RalphContainer container)
    {
        container.getDrive()
                .setDefaultCommand(container.driveByJoystick(processJoystickInput(driverController::getLeftY),
                        processJoystickInput(driverController::getLeftX),
                        processJoystickInput(driverController::getRightX)));

        driverController.back().onTrue(
                container.getDrive().resetOrientation(FieldConstants.getFieldRotation(FieldConstants.getAlliance())));

        driverController.x().whileTrue(container.getDrive().xLock());

        driverController.y().onTrue(Commands.runOnce(() -> container.getDrive()
                .resetPose(FieldConstants.convertPoseByAlliance(FieldConstants.ReefPositions.AB_ALGAE))));

        driverController.rightBumper().whileTrue(container.getGoToReefPoseCommandRight());
        driverController.leftBumper().whileTrue(container.getGoToReefPoseCommandLeft());
    }

    static void bindRollers(CommandXboxController driverController, CommandXboxController manipulatorController,
            RalphContainer container)
    {
        manipulatorController.back().onTrue(Commands.runOnce(() -> container.getDashboard().toggleBeamBreak()));

        container.getRollers().setDefaultCommand(container.getRollers().getStopCommand());

        driverController.rightTrigger().whileTrue(container.getOuttakeCommand());

        manipulatorController.rightTrigger().whileTrue(container.getOuttakeCommand());

        manipulatorController.back().whileTrue(container.getOuttakeCommand(1.0));

        container.getRollers().intakeTrigger().onTrue(new RumbleXBoxController(manipulatorController, 0.5, 0.5)
                .alongWith(new RumbleXBoxController(driverController, 0.5, 0.5)));
    }

    static void bindClimber(CommandXboxController driverController, RalphContainer container)
    {
        container.getClimber().setDefaultCommand(container.getClimber().getStopCommand());
        driverController.a().whileTrue(container.getClimber().getClimbExtendCommand());
        driverController.b().whileTrue(container.getClimber().getClimbRetractCommand());
    }

    static void bindSuperstructure(CommandXboxController driverController, CommandXboxController manipulatorController,
            RalphContainer container)
    {
        container.getSuperstructure().getInnerElevator().setDefaultCommand(container.getSuperstructure()
                .getInnerElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getRightY)));

        container.getSuperstructure().getOuterElevator().setDefaultCommand(container.getSuperstructure()
                .getOuterElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getLeftY)));

        driverController.start()
                .whileTrue(container.getSuperstructure().getHomingCommand(
                        RalphConstants.InnerElevatorConstants.HOMING_SPEED,
                        RalphConstants.OuterElevatorConstants.HOMING_SPEED));

        manipulatorController.start()
                .whileTrue(container.getSuperstructure().getHomingCommand(
                        RalphConstants.InnerElevatorConstants.HOMING_SPEED,
                        RalphConstants.OuterElevatorConstants.HOMING_SPEED));

        manipulatorController.povLeft()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.L1));
        manipulatorController.povUp()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.L2));
        manipulatorController.povRight()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.L3));
        manipulatorController.povDown()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.L4));
        manipulatorController.a().onTrue(Commands.either(
                container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.CORAL_STATION_PRE)
                        .andThen(container.getSuperstructure().getHomingCommand(0.5, 0.5))
                        .andThen(container.getSuperstructure()
                                .getGoToGoalCommand(RalphConstants.SuperstructureGoal.CORAL_STATION)),
                Commands.none(), () -> container.getSuperstructure().getGoal() != SuperstructureGoal.CORAL_STATION)
                .withTimeout(1.5));
        manipulatorController.b().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.PROCESSOR_STATION));
        manipulatorController.y().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.LOWER_ALGAE));
        manipulatorController.x().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.HIGHER_ALGAE));
    }

    @Override
    public void bindOI(RalphContainer container)
    {
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController manipulatorController = new CommandXboxController(1);

        bindDrive(driverController, container);
        bindRollers(driverController, manipulatorController, container);
        bindClimber(driverController, container);
        bindSuperstructure(driverController, manipulatorController, container);

    }
}
