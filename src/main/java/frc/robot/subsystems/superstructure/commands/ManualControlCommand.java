package frc.robot.subsystems.superstructure.commands;

import frc.robot.subsystems.superstructure.Superstructure;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.function.DoubleSupplier;

public class ManualControlCommand extends ParallelCommandGroup
{
    public ManualControlCommand(Superstructure superstructure, DoubleSupplier innerElevatorSpeed,
            DoubleSupplier outerElevatorSpeed)
    {
        addRequirements(superstructure);
        addCommands(superstructure.getInnerElevator().getMoveByJoystick(innerElevatorSpeed),
                superstructure.getOuterElevator().getMoveByJoystick(outerElevatorSpeed));
    }
}
