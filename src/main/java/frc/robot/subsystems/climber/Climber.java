package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

/**
 * Climber subsystem for the robot.
 */

public class Climber extends SubsystemBase
{
    private ClimberIO io;
    private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();
    private final Angle sweetSpot;
    private final Angle stowPosition;
    private final Angle extendPosition;
    private final double climbSpeed;

    /**
     * Constructor for the Climber class.
     * 
     * @param climberIO IO for the climber
     */

    public Climber(ClimberIO climberIO, Angle sweetSpot, Angle stowPosition, Angle extendPosition, double climbSpeed)
    {
        io = climberIO;
        this.sweetSpot = sweetSpot;
        this.stowPosition = stowPosition;
        this.extendPosition = extendPosition;
        this.climbSpeed = climbSpeed;
    }

    /**
     * climb up method for the Climber class.
     * 
     * @param climbSpeed speed to climb up
     */

    public void climbExtend()
    {
        io.run(-climbSpeed);
    }

    /**
     * climb down method for the Climber class.
     * 
     * @param climbSpeed speed to climb down
     */

    public void climbRetract()
    {
        io.run(climbSpeed);
    }

    /**
     * get climb up command method for the Climber class.
     * 
     * @param climbSpeed speed to climb up
     * @return command to climb up
     */

    public Command getClimbExtendCommand()
    {
        return run(() -> climbExtend());
    }

    /**
     * get climb down command method for the Climber class.
     * 
     * @param climbSpeed speed to climb down
     * @return command to climb down
     */

    public Command getClimbRetractCommand()
    {
        return run(() -> climbRetract());
    }

    public class ClimbToPosition extends Command
    {
        private final Angle position;

        public ClimbToPosition(Angle position)
        {
            addRequirements(Climber.this);
            this.position = position;
        }

        @Override
        public void initialize()
        {
            runTo(position);
        }

        @Override
        public boolean isFinished()
        {
            return isAtAngle(position);
        }
    }

    /**
     * climb to perfect position
     */

    public Command climbToPosition(Angle position)
    {
        return new ClimbToPosition(position);
    }

    /**
     * returns a command that runs the climber to the sweet spot
     */

    public Command getRunToSweetSpotCommand()
    {
        return climbToPosition(sweetSpot);
    }

    /**
     * returns a command that runs the climber to the top
     */

    public Command getClimbExtendFully()
    {
        return climbToPosition(extendPosition);
    }

    public Command getStowCommand()
    {
        return climbToPosition(stowPosition);
    }

    /**
     * runs to angle
     * 
     * @param angle angle to run to
     */

    public void runTo(Angle angle)
    {
        io.runTo(angle);
    }

    /**
     * stop method for the Climber class.
     */

    public void stop()
    {
        io.stop();
    }

    /**
     * get stop command method for the Climber class.
     * 
     * @return command to stop
     */

    public Command getStopCommand()
    {
        return run(this::stop);
    }

    public boolean isAtAngle(Angle angle)
    {
        return m_inputs.position.isNear(angle, Degrees.of(5));
    }

    /**
     * periodic method for the Climber class.
     */

    @Override
    public void periodic()
    {
        io.updateInputs(m_inputs);
        Logger.processInputs(getName(), m_inputs);
    }
}
