package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.superstructure.Superstructure;

public class HomingCommand extends ParallelCommandGroup
{

    public HomingCommand(Superstructure superstructure, double innerElevatorSpeed, double outerElevatorSpeed)
    {
        addRequirements(superstructure);
        addCommands(superstructure.getInnerElevator().getHomingCommand(innerElevatorSpeed),
                superstructure.getOuterElevator().getHomingCommand(outerElevatorSpeed));
    }
}