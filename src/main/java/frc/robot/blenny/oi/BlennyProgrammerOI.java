package frc.robot.blenny.oi;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.blenny.BlennyContainer;
import frc.robot.blenny.constants.BlennyConstants;

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

        driverController.b()
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

        driverController.rightTrigger()
                .whileTrue(container.getRollers().getOuttakeCommand(BlennyConstants.RollersConstants.OUTTAKE_SPEED));

        container.getSuperstructure().getWrist()
                .setDefaultCommand(container.getSuperstructure().getWrist().getStopCommand());

        manipulatorController.leftBumper().whileTrue(container.getSuperstructure().getWrist()
                .getSetSpeedCommand(-BlennyConstants.WristJointConstants.MANUAL_MOVE_SPEED));

        manipulatorController.rightBumper().whileTrue(container.getSuperstructure().getWrist()
                .getSetSpeedCommand(BlennyConstants.WristJointConstants.MANUAL_MOVE_SPEED));

        container.getSuperstructure().getInnerElevator().setDefaultCommand(container.getSuperstructure()
                .getInnerElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getRightY)));

        container.getSuperstructure().getOuterElevator().setDefaultCommand(container.getSuperstructure()
                .getOuterElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getLeftY)));

        driverController.start()
                .whileTrue(container.getSuperstructure().getHomingCommand(
                        BlennyConstants.InnerElevatorConstants.HOMING_SPEED,
                        BlennyConstants.OuterElevatorConstants.HOMING_SPEED));

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

        container.getRollers().setDefaultCommand(container.getRollers().getStopCommand());

        container.getClimber().setDefaultCommand(container.getClimber().getStopCommand());

        container.getDashboard().setInnerElevatorGoToPosition(container.getSuperstructure().getInnerElevator()
                .getMoveToPositionCommand(container.getDashboard().getInnerElevatorTargetPosition()));
        container.getDashboard().setOuterElevatorGoToPosition(container.getSuperstructure().getOuterElevator()
                .getMoveToPositionCommand(container.getDashboard().getOuterElevatorTargetPosition()));
        driverController.rightBumper().whileTrue(Commands.defer(
                () -> container.getDrive()
                        .driveToPose(FieldConstants.getReefBackupPosition(container.getDashboard().getTargetPose(),
                                Feet.of(1)))
                        .alongWith(container.getSuperstructure()
                                .getGoToGoalCommand(container.getDashboard().getSuperstructureGoal()))
                        .andThen(() -> container.getDrive().driveToPose(container.getDashboard().getTargetPose())),
                Set.of(container.getSuperstructure())));

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
