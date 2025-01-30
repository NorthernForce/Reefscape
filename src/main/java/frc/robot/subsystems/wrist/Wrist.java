package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
}