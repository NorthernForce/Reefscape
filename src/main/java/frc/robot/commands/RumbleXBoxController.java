package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Rumbles the Xbox controller for a specified amount of time.
 */
public class RumbleXBoxController extends Command
{
    private final XboxController controller;
    private final Timer timer = new Timer();
    private final double rumble;
    private final Time time;

    /**
     * Rumbles the Xbox controller for a specified amount of time.
     * 
     * @param controller the Xbox controller to rumble
     * @param rumble     the rumble value (between 0 and 1)
     * @param time       the time to rumble for
     */
    public RumbleXBoxController(CommandXboxController controller, double rumble, Time time)
    {
        this.controller = controller.getHID();
        this.rumble = rumble;
        this.time = time;
    }

    /**
     * Rumbles the Xbox controller for a specified amount of time.
     * 
     * @param controller the Xbox controller to rumble
     * @param rumble     the rumble value (between 0 and 1)
     * @param time       the time to rumble for (in seconds)
     */
    public RumbleXBoxController(CommandXboxController controller, double rumble, double time)
    {
        this(controller, rumble, Seconds.of(time));
    }

    @Override
    public void initialize()
    {
        timer.restart();
        controller.setRumble(RumbleType.kBothRumble, rumble);
    }

    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(time.in(Seconds));
    }

    @Override
    public void end(boolean interrupted)
    {
        controller.setRumble(RumbleType.kBothRumble, 0);
    }
}
