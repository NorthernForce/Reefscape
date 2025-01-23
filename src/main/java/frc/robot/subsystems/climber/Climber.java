package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Climber implements Subsystem
{
	private ClimberIO io;
	private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();

	public Climber(ClimberIO climberIO)
	{
		io = climberIO;
	}

	public void climbUp()
	{
		io.climbUp();
	}

	public void climbDown()
	{
		io.climbDown();
	}

	public Command getClimbUpCommand()
	{
		return new Command()
		{
			@Override
			public void execute()
			{
				climbUp();
			}

			@Override
			public boolean isFinished()
			{
				return true;
			}
		};
	}

	public Command getClimbDownCommand()
	{
		return new Command()
		{
			@Override
			public void execute()
			{
				climbDown();
			}

			@Override
			public boolean isFinished()
			{
				return true;
			}
		};
	}

	public void stop()
	{
		io.stop();
	}

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

	@Override
	public void periodic()
	{
		io.updateInputs(m_inputs);
		Logger.processInputs(getName(), m_inputs);
	}
}
