package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;
import frc.robot.subsystems.superstructure.Superstructure;

public class HoldAtGoalCommand extends Command
{
    private final SuperstructureGoal goal;
    private final Superstructure superstructure;

    public HoldAtGoalCommand(Superstructure superstructure, SuperstructureGoal goal)
    {
        addRequirements(superstructure);
        this.goal = goal;
        this.superstructure = superstructure;
    }

    @Override
    public void initialize()
    {
        superstructure.setGoal(goal);
        superstructure.getInnerElevator().setTargetPosition(goal.getInnerElevatorGoal());
        superstructure.getOuterElevator().setTargetPosition(goal.getOuterElevatorGoal());
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
