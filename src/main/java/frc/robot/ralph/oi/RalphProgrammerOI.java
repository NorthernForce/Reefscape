package frc.robot.ralph.oi;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.ralph.RalphContainer;

/**
 * Ralph OI for the driver and operator
 */
public class RalphProgrammerOI implements RalphOI
{
    @Override
    public void bindOI(RalphContainer container)
    {
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController manipulatorController = new CommandXboxController(1);

        RalphDriverOI.bindDrive(driverController, container);
        RalphDriverOI.bindInserter(driverController, manipulatorController, container);
        RalphDriverOI.bindClimber(driverController, container);
        RalphDriverOI.bindSuperstructure(driverController, manipulatorController, container);
        RalphDriverOI.bindAlgaeRemover(manipulatorController, container);

        manipulatorController.leftStick().whileTrue(
                Commands.sequence(container.getSuperstructure().getOuterElevator().getSysIdQuasistaicForward(),
                        container.getSuperstructure().getOuterElevator().getSysIdQuasistaicReverse(),
                        container.getSuperstructure().getOuterElevator().getSysIdDynamicForward(),
                        container.getSuperstructure().getOuterElevator().getSysIdDynamicReverse()));

        manipulatorController.rightStick().whileTrue(
                Commands.sequence(container.getSuperstructure().getInnerElevator().getSysIdQuasistaicForward(),
                        container.getSuperstructure().getInnerElevator().getSysIdQuasistaicReverse(),
                        container.getSuperstructure().getInnerElevator().getSysIdDynamicForward(),
                        container.getSuperstructure().getInnerElevator().getSysIdDynamicReverse()));

        driverController.leftStick().whileTrue(Commands.sequence(Commands.runOnce(SignalLogger::start),
                container.getDrive().sysIdTranslationQuasistatic(Direction.kForward),
                container.getDrive().sysIdTranslationQuasistatic(Direction.kReverse),
                container.getDrive().sysIdTranslationDynamic(Direction.kForward),
                container.getDrive().sysIdTranslationDynamic(Direction.kReverse),
                container.getDrive().sysIdRotationQuasistatic(Direction.kForward),
                container.getDrive().sysIdRotationQuasistatic(Direction.kReverse),
                container.getDrive().sysIdRotationDynamic(Direction.kForward),
                container.getDrive().sysIdRotationDynamic(Direction.kReverse),
                container.getDrive().sysIdSteerQuasistatic(Direction.kForward),
                container.getDrive().sysIdSteerQuasistatic(Direction.kReverse),
                container.getDrive().sysIdSteerDynamic(Direction.kForward),
                container.getDrive().sysIdSteerDynamic(Direction.kReverse), Commands.runOnce(SignalLogger::stop)));
    }
}
