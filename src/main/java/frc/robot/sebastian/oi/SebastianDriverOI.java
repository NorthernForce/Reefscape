package frc.robot.sebastian.oi;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.sebastian.SebastianContainer;
import frc.robot.sebastian.constants.SebastianConstants;
import frc.robot.sebastian.constants.SebastianConstants.SuperstructureGoal;
import static edu.wpi.first.units.Units.*;

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
            double x = MathUtil.applyDeadband(input.getAsDouble(), 0.0, 1);
            return -x * Math.abs(x);
        };
    }

    static void bindDrive(CommandXboxController driverController, SebastianContainer container)
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
                                            .lte(SebastianConstants.DrivetrainConstants.SAFE_DISTANCE))
                                    {
                                        return 0;
                                    }
                                    return 1;
                                }, () -> -1, () -> 1, () -> -1));

        driverController.rightBumper().whileTrue(Commands.either(
                Commands.defer(() -> container.getDrive()
                        .driveToPose(FieldConstants.getReefBackupPosition(container.getDashboard().getTargetPose(),
                                Feet.of(1)))
                        .alongWith(container.getSuperstructure()
                                .getGoToGoalCommand(container.getDashboard().getSuperstructureGoalForReef()))
                        .andThen(() -> container.getDrive().driveToPose(container.getDashboard().getTargetPose())), Set.of()),
                Commands.defer(() -> container.getDrive()
                        .driveToPose(container.getDashboard().getStationTargetPose())
                        .alongWith(container.getSuperstructure()
                                .getGoToGoalCommand(container.getDashboard().getSuperstructureGoalForStation())), Set.of()),
                () -> !(container.getRollers().hasAlgae() || container.getRollers().hasCoral())));
    }

    static void bindRollers(CommandXboxController driverController, CommandXboxController manipulatorController,
            SebastianContainer container)
    {
        container.getRollers().setDefaultCommand(container.getRollers().getStopCommand());

        driverController.leftTrigger()
                .whileTrue(container.getIntakeCommand().andThen(rumble(driverController)));

        driverController.rightTrigger().whileTrue(container.getOuttakeCommand());

        manipulatorController.leftTrigger()
                .whileTrue(container.getIntakeCommand().andThen(rumble(manipulatorController)));
        
        manipulatorController.rightTrigger().whileTrue(container.getOuttakeCommand());
        
        new Trigger(() -> container.getRollers().hasAlgae() && !container.getRollers().hasCoral())
                .whileTrue(container.getStowCommand());
    }

    static void bindClimber(CommandXboxController driverController, SebastianContainer container)
    {
        container.getClimber().setDefaultCommand(container.getClimber().getStopCommand());
        driverController.a().whileTrue(container.getClimber().getRunToSweetSpotCommand());
        driverController.b().onTrue(container.getClimber().getClimbExtend());
    }

    static void bindSuperstructure(CommandXboxController driverController, CommandXboxController manipulatorController,
            SebastianContainer container)
    {
        container.getSuperstructure().getWrist()
                .setDefaultCommand(container.getSuperstructure().getWrist().getStopCommand());

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

        manipulatorController.leftBumper().whileTrue(container.getSuperstructure().getWrist()
                .getSetSpeedCommand(-SebastianConstants.WristJointConstants.MANUAL_MOVE_SPEED));

        manipulatorController.rightBumper().whileTrue(container.getSuperstructure().getWrist()
                .getSetSpeedCommand(SebastianConstants.WristJointConstants.MANUAL_MOVE_SPEED));

        manipulatorController.povLeft()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.L1));
        manipulatorController.povUp()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.L2));
        manipulatorController.povRight()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.L3));
        manipulatorController.povDown()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.L4));
        manipulatorController.a().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.CORAL_STATION));
        manipulatorController.b().whileTrue(container.getSuperstructure()
                .getGoToGoalCommand(SebastianConstants.SuperstructureGoal.PROCESSOR_STATION));
        manipulatorController.y().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.LOWER_ALGAE));
        manipulatorController.x().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(SebastianConstants.SuperstructureGoal.HIGHER_ALGAE));
    }

    public static Command rumble(CommandXboxController controller)
    {
        return Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0))
                .andThen(Commands.waitSeconds(0.5))
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
