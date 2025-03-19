package frc.robot.subsystems.algaeremover.commands;

import frc.robot.subsystems.algaeremover.AlgaeRemover;
import edu.wpi.first.wpilibj2.command.Command;

public class RemoveAlgaeCommand extends Command
{
    private AlgaeRemover algaeRemover;
    private double speed;

    public RemoveAlgaeCommand(AlgaeRemover algaeRemover, double speed)
    {
        this.algaeRemover = algaeRemover;
        this.speed = speed;
        addRequirements(algaeRemover);
    }

    @Override
    public void initialize()
    {
        algaeRemover.set(speed);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        algaeRemover.stop();
    }
}