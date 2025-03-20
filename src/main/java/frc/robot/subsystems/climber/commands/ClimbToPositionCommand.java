package frc.robot.subsystems.climber.commands;

import frc.robot.subsystems.climber.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Angle;

public class ClimbToPositionCommand extends Command
{
    private Climber climber;
    private final Angle position;

    public ClimbToPositionCommand(Climber climber, Angle position)
    {
        this.climber = climber;
        this.position = position;
        addRequirements(climber);
    }

    @Override
    public void initialize()
    {
        climber.runTo(position);
    }

    @Override
    public boolean isFinished()
    {
        return climber.isAtAngle(position);
    }

    @Override
    public void end(boolean interrupted)
    {
        climber.stop();
    }
}