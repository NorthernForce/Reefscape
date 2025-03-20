package frc.robot.subsystems.superstructure.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class ElevatorHomingCommand extends Command
{
    private double speed;
    private Elevator elevator;

    public ElevatorHomingCommand(Elevator elevator, double speed)
    {
        addRequirements(elevator);
        this.speed = speed;
        this.elevator = elevator;
    }

    @Override
    public void initialize()
    {
        elevator.getElevatorIO().setLowerLimitEnable(false);
    }

    @Override
    public void execute()
    {
        elevator.getElevatorIO().setSpeed(-speed, true);
    }

    @Override
    public boolean isFinished()
    {
        return elevator.getInputs().isAtBottom;
    }

    @Override
    public void end(boolean isFinished)
    {
        elevator.getElevatorIO().stop();
        elevator.getElevatorIO().resetPosition();
        elevator.getElevatorIO().setLowerLimitEnable(true);
    }
}
