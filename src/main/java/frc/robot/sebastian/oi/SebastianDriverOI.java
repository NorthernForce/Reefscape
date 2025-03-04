package frc.robot.sebastian.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.sebastian.SebastianContainer;
import frc.robot.sebastian.constants.SebastianConstants;
import frc.robot.sebastian.constants.SebastianConstants.SuperstructureGoal;

/**
 * Sebastian OI for the driver and operator
 */
public class SebastianDriverOI implements SebastianOI
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

    static void bindDrive(CommandXboxController driverController, SebastianContainer container)
    {
        container.getDrive()
                .setDefaultCommand(container.getDriveByJoystickCommand(processJoystickInput(driverController::getLeftY),
                        processJoystickInput(driverController::getLeftX),
                        processJoystickInput(driverController::getRightX)));

        new Trigger(() -> container.getSuperstructure().isTooHigh())
                .whileTrue(container.getDriveByJoystickWithSlewCommand(processJoystickInput(driverController::getLeftY),
                        processJoystickInput(driverController::getLeftX),
                        processJoystickInput(driverController::getRightX)));

        driverController.back().onTrue(container.getDrive()
                .getResetOrientationCommand(FieldConstants.getFieldRotation(FieldConstants.getAlliance())));

        driverController.x().whileTrue(container.getDrive().getXLockCommand());

        new Trigger(() -> !container.getSuperstructure().isAtGoal()
                && container.getSuperstructure().getGoal() == SuperstructureGoal.CORAL_STATION
                && container.getViewer().isPresent()
                && FieldConstants.isAtCoralRotation(container.getDrive().getPose().getRotation())
                && !driverController.getHID().getLeftBumperButton()).whileTrue(
                        container.getDriveByJoystickWithLimitsCommand(processJoystickInput(driverController::getLeftY),
                                processJoystickInput(driverController::getLeftX),
                                processJoystickInput(driverController::getRightX)));

        driverController.rightBumper().whileTrue(container.getDrive().getGoLeft(-0.325));
        driverController.leftBumper().whileTrue(container.getDrive().getGoLeft(0.325));
    }

    static void bindRollers(CommandXboxController driverController, CommandXboxController manipulatorController,
            SebastianContainer container)
    {
        manipulatorController.back().onTrue(Commands.runOnce(() -> container.getDashboard().toggleBeamBreak()));

        container.getRollers().setDefaultCommand(container.getRollers().getStopCommand());

        driverController.leftTrigger().whileTrue(container.getIntakeCommand().andThen(rumble(driverController))
                .onlyIf(() -> SmartDashboard.getBoolean("Use Beam Break", true)));

        driverController.rightTrigger().whileTrue(container.getOuttakeCommand());

        manipulatorController.leftTrigger()
                .whileTrue((container.getIntakeCommand()).andThen(rumble(manipulatorController))
                        .onlyIf(() -> SmartDashboard.getBoolean("Use Beam Break", true)));

        manipulatorController.rightTrigger().whileTrue(container.getOuttakeCommand());

        manipulatorController.back().whileTrue(container.getOuttakeCommand(1.0));

        // new Trigger(() -> container.getRollers().hasAlgae() &&
        // !container.getRollers().hasCoral()
        // && (container.getSuperstructure().getGoal() ==
        // SuperstructureGoal.HIGHER_ALGAE
        // || container.getSuperstructure().getGoal() ==
        // SuperstructureGoal.LOWER_ALGAE))
        // .onTrue(container.getStowCommand().until(() -> container.getSuperstructure()
        // .getGoal() == SuperstructureGoal.PROCESSOR_STATION));
    }

    static void bindClimber(CommandXboxController driverController, SebastianContainer container)
    {
        container.getClimber().setDefaultCommand(container.getClimber().getStopCommand());
        driverController.a().whileTrue(container.getClimber().getClimbExtendCommand());
        driverController.b().whileTrue(container.getClimber().getClimbRetractCommand());
    }

    static void bindSuperstructure(CommandXboxController driverController, CommandXboxController manipulatorController,
            SebastianContainer container)
    {
        container.getSuperstructure().getInnerElevator().setDefaultCommand(container.getSuperstructure()
                .getInnerElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getRightY)));

        container.getSuperstructure().getOuterElevator().setDefaultCommand(container.getSuperstructure()
                .getOuterElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getLeftY)));

        driverController.start()
                .whileTrue(container.getSuperstructure().getHomingCommand(
                        SebastianConstants.InnerElevatorConstants.HOMING_SPEED,
                        SebastianConstants.OuterElevatorConstants.HOMING_SPEED));

        manipulatorController.start()
                .whileTrue(container.getSuperstructure().getHomingCommand(
                        SebastianConstants.InnerElevatorConstants.HOMING_SPEED,
                        SebastianConstants.OuterElevatorConstants.HOMING_SPEED));

        manipulatorController.povLeft()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.L1));
        manipulatorController.povUp()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.L2));
        manipulatorController.povRight()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.L3));
        manipulatorController.povDown()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.L4));
        manipulatorController.a().onTrue(Commands
                .either(container.getSuperstructure()
                        .getGoToGoalCommand(SebastianConstants.SuperstructureGoal.CORAL_STATION_PRE)
                        .andThen(container.getSuperstructure().getHomingCommand(0.5, 0.5))
                        .andThen(container.getSuperstructure()
                                .getGoToGoalCommand(SebastianConstants.SuperstructureGoal.CORAL_STATION)),
                        Commands.none(),
                        () -> container.getSuperstructure().getGoal() != SuperstructureGoal.CORAL_STATION)
                .withTimeout(1.5));
        manipulatorController.b().whileTrue(container.getSuperstructure()
                .getGoToGoalCommand(SebastianConstants.SuperstructureGoal.PROCESSOR_STATION));
        manipulatorController.y().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.LOWER_ALGAE));
        manipulatorController.x().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.HIGHER_ALGAE));
    }

    public static Command rumble(CommandXboxController controller)
    {
        return Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.5))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
    }

    @Override
    public void bindOI(SebastianContainer container)
    {
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController manipulatorController = new CommandXboxController(1);

        bindDrive(driverController, container);
        bindRollers(driverController, manipulatorController, container);
        bindClimber(driverController, container);
        bindSuperstructure(driverController, manipulatorController, container);

    }
}
