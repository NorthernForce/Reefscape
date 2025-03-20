package frc.robot.subsystems.superstructure.elevator.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class ElevatorMoveToPositionCommand extends Command
{
    private Distance position;
    private final Elevator elevator;

    public ElevatorMoveToPositionCommand(Elevator elevator, Distance position)
    {
        addRequirements(elevator);
        this.position = position;
        this.elevator = elevator;
    }

    @Override
    public void initialize()
    {
        elevator.setTargetPosition(position);
    }

    @Override
    public boolean isFinished()
    {
        return elevator.isAtTargetPosition();
    }

    @Override
    public void end(boolean interrupted)
    {
        elevator.stop();
    }
}
