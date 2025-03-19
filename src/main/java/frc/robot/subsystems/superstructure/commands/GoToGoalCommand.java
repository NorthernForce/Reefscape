package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;

public class GoToGoalCommand extends Command
{
    private final SuperstructureGoal goal;
    private final Superstructure superstructure;

    public GoToGoalCommand(Superstructure superstructure, SuperstructureGoal goal)
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
        return superstructure.getInnerElevator().isAtTargetPosition()
                && superstructure.getOuterElevator().isAtTargetPosition();
    }
}
