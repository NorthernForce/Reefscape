package frc.robot.subsystems.inserter.commands;

import frc.robot.subsystems.inserter.Inserter;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralOuttakeCommand extends Command
{
    private Inserter inserter;

    public CoralOuttakeCommand(Inserter inserter)
    {
        this.inserter = inserter;
        addRequirements(inserter);
    }

    @Override
    public void initialize()
    {
        inserter.outtake();
    }

    @Override
    public boolean isFinished()
    {
        return !inserter.hasCoral();
    }

    @Override
    public void end(boolean interrupted)
    {
        inserter.stop();
    }
}