package frc.robot.ralph.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.commands.RumbleXBoxController;
import frc.robot.ralph.RalphContainer;

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

        driverController.leftBumper().whileTrue(container.driveToLeftReef());

        driverController.rightBumper().whileTrue(container.driveToRightReef());

        SmartDashboard.putData("DriveToReef", container.driveToLeftReef());
    }

    static void bindInserter(CommandXboxController driverController, CommandXboxController manipulatorController,
            RalphContainer container)
    {
        container.getInserter().setDefaultCommand(container.defaultIntake());

        driverController.rightTrigger().and(container.getSuperstructure()::isAtGoal)
                .whileTrue(container.outtakeCoral());

        manipulatorController.rightTrigger().and(container.getSuperstructure()::isAtGoal)
                .whileTrue(container.outtakeCoral());

        container.getInserter().intakeTrigger().onTrue(new RumbleXBoxController(manipulatorController, 0.5, 0.5)
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

        manipulatorController.a().onTrue(container.goToIntake());

        driverController.start().whileTrue(container.homeElevator());

        manipulatorController.start().whileTrue(container.homeElevator());

        manipulatorController.povLeft().and(container.getInserter()::hasCoral).onTrue(container.goToL1());
        manipulatorController.povUp().and(container.getInserter()::hasCoral).onTrue(container.goToL2());
        manipulatorController.povRight().and(container.getInserter()::hasCoral).onTrue(container.goToL3());
        manipulatorController.povDown().and(container.getInserter()::hasCoral).onTrue(container.goToL4());

        container.getSuperstructure()
                .setDefaultCommand(container.getSuperstructure().getManualControlCommand(
                        processJoystickInput(manipulatorController::getRightY),
                        processJoystickInput(manipulatorController::getLeftY)));
    }

    @Override
    public void bindOI(RalphContainer container)
    {
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController manipulatorController = new CommandXboxController(1);

        bindDrive(driverController, container);
        bindInserter(driverController, manipulatorController, container);
        bindClimber(driverController, container);
        bindSuperstructure(driverController, manipulatorController, container);

    }
}
