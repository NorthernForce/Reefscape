package frc.robot.ralph.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ralph.RalphContainer;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;

public class IntakeWhileWaitingCommand extends Command
{
    private RalphContainer container;

    public IntakeWhileWaitingCommand(RalphContainer container)
    {
        addRequirements(container.getInserter());
        this.container = container;
    }

    @Override
    public void execute()
    {
        if (!container.getInserter().hasCoral()
                && container.getSuperstructure().isAtGoal(SuperstructureGoal.CORAL_STATION))
        {
            container.getInserter().intake();
        } else
        {
            container.getInserter().stop();
        }
    }
}
