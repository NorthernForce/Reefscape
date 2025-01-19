package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Elevator implements Subsystem
{
	private ElevatorIO m_motor;
	private ElevatorState m_level = ElevatorState.L1;
	private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();

	public Elevator(ElevatorIO io)
	{
		m_motor = io;
	}

	public void setLevel(ElevatorState level)
	{
		m_level = level;
	}

	private void start(double speed, ElevatorState level)
	{
		if (level.getLevel() > m_level.getLevel())
		{
			m_motor.setInverted(false);
		} else
		{
			m_motor.setInverted(true);
		}
		m_motor.start(speed);
	}

	public void startInner(double speed, ElevatorState level)
	{
		if (level.getLevel() > m_level.getLevel())
		{
			m_motor.setInnerInverted(false);
		} else
		{
			m_motor.setInnerInverted(true);
		}
		m_motor.startInner(speed);
	}

	public boolean passedLevel(ElevatorState level, ElevatorState previousLevel)
	{
		if (level.getLevel() > previousLevel.getLevel())
		{
			if (level.getLevel() <= 2)
			{
				return m_inputs.position.abs(Meters) >= ElevatorState.L1.getHeight();
			} else
			{
				return m_inputs.position.abs(Meters) >= ElevatorState.L3.getHeight();
			}
		} else
		{
			if (level.getLevel() <= 2)
			{
				return m_inputs.position.abs(Meters) <= ElevatorState.L1.getHeight();
			} else
			{
				return m_inputs.position.abs(Meters) <= ElevatorState.L3.getHeight();
			}
		}
	}

	public boolean passedInnerLevel(ElevatorState level, ElevatorState previousLevel)
	{
		if (level.getLevel() > previousLevel.getLevel())
		{
			if (level.getLevel() <= 2)
			{
				return m_inputs.innerPosition.abs(Meters) + ElevatorState.L1.getHeight() >= level.getHeight();
			} else
			{
				return m_inputs.innerPosition.abs(Meters) + ElevatorState.L3.getHeight() >= level.getHeight();
			}
		} else
		{
			if (level.getLevel() <= 2)
			{
				return m_inputs.innerPosition.abs(Meters) + ElevatorState.L1.getHeight() <= level.getHeight();
			} else
			{
				return m_inputs.innerPosition.abs(Meters) + ElevatorState.L3.getHeight() <= level.getHeight();
			}
		}
	}

	private void stop()
	{
		m_motor.stopInner();
		m_motor.stop();
	}

	public Command getLevelCommand(ElevatorState level)
	{

		return new Command()
		{
			@Override
			public void initialize()
			{
				start(0.5, level);
			}

			@Override
			public boolean isFinished()
			{
				return passedLevel(level, m_level) && passedInnerLevel(level, m_level);
			}

			@Override
			public void end(boolean isInterrupted)
			{
				stop();
				m_level = level;
			}
		};
	}

	public Command getL1Command()
	{
		return getLevelCommand(ElevatorState.L1);
	}

	public Command getL2Command()
	{
		return getLevelCommand(ElevatorState.L2);
	}

	public Command getL3Command()
	{
		return getLevelCommand(ElevatorState.L3);
	}

	public Command getL4Command()
	{
		return getLevelCommand(ElevatorState.L4);
	}

	@Override
	public void periodic()
	{
		m_motor.updateInputs(m_inputs);
		m_inputs.state = m_level;
		Logger.processInputs(getName(), m_inputs);
	}
}
