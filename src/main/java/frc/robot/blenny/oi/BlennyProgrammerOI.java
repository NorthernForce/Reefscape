package frc.robot.blenny.oi;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.blenny.BlennyContainer;
import frc.robot.blenny.constants.BlennyConstants;
import frc.robot.blenny.constants.BlennyConstants.SuperstructureGoal;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

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

    private void bindDrive(CommandXboxController driverController, BlennyContainer container)
    {
        container.getDrive().setDefaultCommand(container.getDrive().getDriveByJoystickCommand(
                processJoystickInput(driverController::getLeftY), processJoystickInput(driverController::getLeftX),
                processJoystickInput(driverController::getRightX)));

        driverController.back().onTrue(container.getDrive()
                .getResetOrientationCommand(FieldConstants.getFieldRotation(FieldConstants.getAlliance())));

        driverController.x().whileTrue(container.getDrive().getXLockCommand());

        new Trigger(() -> !container.getSuperstructure().isAtGoal()
                && container.getSuperstructure().getGoal() == SuperstructureGoal.CORAL_STATION
                && container.getViewer().isPresent()
                && FieldConstants.isAtCoralRotation(container.getDrive().getPose().getRotation()))
                        .whileTrue(container.getDrive().getDriveByJoystickWithRobotRelativeLimits(
                                processJoystickInput(driverController::getLeftY),
                                processJoystickInput(driverController::getLeftX),
                                processJoystickInput(driverController::getRightX), () ->
                                {
                                    if (container.getViewer().getCenterDistance()
                                            .lte(BlennyConstants.DrivetrainConstants.SAFE_DISTANCE))
                                    {
                                        return 0;
                                    }
                                    return 1;
                                }, () -> -1, () -> 1, () -> -1));

        driverController.rightBumper().whileTrue(Commands.defer(
                () -> container.getDrive()
                        .driveToPose(FieldConstants.getReefBackupPosition(container.getDashboard().getTargetPose(),
                                Feet.of(1)))
                        .alongWith(container.getSuperstructure()
                                .getGoToGoalCommand(container.getDashboard().getSuperstructureGoal()))
                        .andThen(() -> container.getDrive().driveToPose(container.getDashboard().getTargetPose())),
                Set.of(container.getSuperstructure())));
    }

    private void bindRollers(CommandXboxController driverController, CommandXboxController manipulatorController,
            BlennyContainer container)
    {
        container.getRollers().setDefaultCommand(container.getRollers().getStopCommand());

        driverController.leftTrigger()
                .whileTrue(Commands.either(
                        container.getRollers().getAlgaeIntakeCommand(),
                        container.getRollers().getCoralIntakeCommand(),
                        () -> container.isInAlgaeState()).andThen(rumble(driverController)));

        driverController.rightTrigger()
                .whileTrue(container.getRollers().getOuttakeCommand());

        manipulatorController.leftTrigger()
                .whileTrue(Commands.either(
                        container.getRollers().getAlgaeIntakeCommand(),
                        container.getRollers().getCoralIntakeCommand(),
                        () -> container.isInAlgaeState()).andThen(rumble(manipulatorController)));

        manipulatorController.rightTrigger()
                .whileTrue(container.getRollers().getOuttakeCommand());
    }

    private void bindClimber(CommandXboxController driverController, BlennyContainer container)
    {
        container.getClimber().setDefaultCommand(container.getClimber().getStopCommand());
        driverController.a().whileTrue(container.getClimber().getRunToSweetSpotCommand());
        driverController.b().onTrue(container.getClimber().getClimbExtend());
    }

    private void bindSuperstructure(CommandXboxController driverController, CommandXboxController manipulatorController,
            BlennyContainer container)
    {
        container.getSuperstructure().getWrist()
                .setDefaultCommand(container.getSuperstructure().getWrist().getStopCommand());

        container.getSuperstructure().getInnerElevator().setDefaultCommand(container.getSuperstructure()
                .getInnerElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getRightY)));

        container.getSuperstructure().getOuterElevator().setDefaultCommand(container.getSuperstructure()
                .getOuterElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getLeftY)));

        driverController.start()
                .whileTrue(container.getSuperstructure().getHomingCommand(
                        BlennyConstants.InnerElevatorConstants.HOMING_SPEED,
                        BlennyConstants.OuterElevatorConstants.HOMING_SPEED));

        manipulatorController.start()
                .whileTrue(container.getSuperstructure().getHomingCommand(
                        BlennyConstants.InnerElevatorConstants.HOMING_SPEED,
                        BlennyConstants.OuterElevatorConstants.HOMING_SPEED));

        manipulatorController.leftBumper().whileTrue(container.getSuperstructure().getWrist()
                .getSetSpeedCommand(-BlennyConstants.WristJointConstants.MANUAL_MOVE_SPEED));

        manipulatorController.rightBumper().whileTrue(container.getSuperstructure().getWrist()
                .getSetSpeedCommand(BlennyConstants.WristJointConstants.MANUAL_MOVE_SPEED));

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
    }

    public Command rumble(CommandXboxController controller)
    {
        return Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.5))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
    }

    @Override
    public void bindOI(BlennyContainer container)
    {
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController manipulatorController = new CommandXboxController(1);

        bindDrive(driverController, container);
        bindRollers(driverController, manipulatorController, container);
        bindClimber(driverController, container);
        bindSuperstructure(driverController, manipulatorController, container);

        driverController.y()
                .whileTrue(Commands.sequence(Commands.runOnce(() -> SignalLogger.start()),
                        container.getDrive().getSysIdTranslationQuasistatic(SysIdRoutine.Direction.kForward),
                        container.getDrive().getSysIdTranslationQuasistatic(SysIdRoutine.Direction.kReverse),
                        container.getDrive().getSysIdTranslationDynamic(SysIdRoutine.Direction.kForward),
                        container.getDrive().getSysIdTranslationDynamic(SysIdRoutine.Direction.kReverse),

                        container.getDrive().getSysIdRotationQuasistatic(SysIdRoutine.Direction.kForward),
                        container.getDrive().getSysIdRotationQuasistatic(SysIdRoutine.Direction.kReverse),
                        container.getDrive().getSysIdRotationDynamic(SysIdRoutine.Direction.kForward),
                        container.getDrive().getSysIdRotationDynamic(SysIdRoutine.Direction.kReverse),

                        container.getDrive().getSysIdSteerQuasistatic(SysIdRoutine.Direction.kForward),
                        container.getDrive().getSysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse),
                        container.getDrive().getSysIdSteerDynamic(SysIdRoutine.Direction.kForward),
                        container.getDrive().getSysIdSteerDynamic(SysIdRoutine.Direction.kReverse),
                        Commands.runOnce(() -> SignalLogger.stop())));

        manipulatorController.leftStick()
                .whileTrue(Commands.sequence(container.getSuperstructure().getOuterElevator().getSysIdDynamicForward(),
                        container.getSuperstructure().getOuterElevator().getSysIdDynamicReverse(),
                        container.getSuperstructure().getOuterElevator().getSysIdQuasistaicForward(),
                        container.getSuperstructure().getOuterElevator().getSysIdQuasistaicReverse()));

        manipulatorController.rightStick()
                .whileTrue(Commands.sequence(container.getSuperstructure().getInnerElevator().getSysIdDynamicForward(),
                        container.getSuperstructure().getInnerElevator().getSysIdDynamicReverse(),
                        container.getSuperstructure().getInnerElevator().getSysIdQuasistaicForward(),
                        container.getSuperstructure().getInnerElevator().getSysIdQuasistaicReverse()));
    }

}
