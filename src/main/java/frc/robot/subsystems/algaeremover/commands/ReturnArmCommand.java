package frc.robot.subsystems.algaeremover.commands;

import frc.robot.subsystems.algaeremover.AlgaeRemover;
import edu.wpi.first.wpilibj2.command.Command;

public class ReturnArmCommand extends Command
{
    private AlgaeRemover algaeRemover;
    private double speed;

    public ReturnArmCommand(AlgaeRemover algaeRemover, double speed)
    {
        this.algaeRemover = algaeRemover;
        this.speed = speed;
        addRequirements(algaeRemover);
    }

    @Override
    public void execute()
    {
        if (!algaeRemover.hasReachedTop())
        {
            algaeRemover.set(-speed);
        }
    }

    @Override
    public boolean isFinished()
    {
        return algaeRemover.hasReachedTop();
    }

    @Override
    public void end(boolean interrupted)
    {
        algaeRemover.stop();
    }
}