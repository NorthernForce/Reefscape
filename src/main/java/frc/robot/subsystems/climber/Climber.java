package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climber subsystem for the robot.
 */

public class Climber extends SubsystemBase
{
	private ClimberIO io;
	private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();

    /**
     * Constructor for the Climber class.
     * @param climberIO IO for the climber
     */

	public Climber(ClimberIO climberIO)
	{
		io = climberIO;
	}

    /**
     * climb up method for the Climber class.
     * @param climbSpeed speed to climb up
     */

	public void climbUp(double climbSpeed)
	{
		io.climbUp(climbSpeed);
	}

    /**
     * climb down method for the Climber class.
     * @param climbSpeed speed to climb down
     */

	public void climbDown(double climbSpeed)
	{
		io.climbDown(climbSpeed);
	}

    /**
     * get climb up command method for the Climber class.
     * @param climbSpeed speed to climb up
     * @return command to climb up
     */

	public Command getClimbUpCommand(double climbSpeed)
	{
		return new Command()
		{
			@Override
			public void execute()
			{
				climbUp(climbSpeed);
			}

			@Override
			public boolean isFinished()
			{
				return true;
			}
		};
	}

    /**
     * get climb down command method for the Climber class.
     * @param climbSpeed speed to climb down
     * @return command to climb down
     */

	public Command getClimbDownCommand(double climbSpeed)
	{
		return new Command()
		{
			@Override
			public void execute()
			{
				climbDown(climbSpeed);
			}

			@Override
			public boolean isFinished()
			{
				return true;
			}
		};
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
     * @return command to stop
     */

	public Command getStopCommand()
	{
		return new Command()
		{
			@Override
			public void execute()
			{
				stop();
			}

			@Override
			public boolean isFinished()
			{
				return true;
			}
		};
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
