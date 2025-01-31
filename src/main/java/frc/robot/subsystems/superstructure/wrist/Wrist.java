package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.superstructure.wrist.WristIO;

public class Wrist extends SubsystemBase
{
    public final WristIO io;

    public Wrist(WristIO io)
    {
        this.io = io;
    }

    public void run(double speed)
    {
    }

    public Command getMoveToAngleCommand(Angle angle)
    {
        return null;
    }

    public Command getStopCommand()
    {
        return null;
    }

    public Angle getAngle()
    {
        return null;
    }

    public boolean isAtTargetPosition()
    {
        return true;
    }

    public boolean isAtPosition(Angle angle)
    {
        return true;
    }
}