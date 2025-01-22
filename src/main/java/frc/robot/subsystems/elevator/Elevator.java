package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Elevator implements Subsystem
{
	private ElevatorIO m_motorOuter;
	private ElevatorIO m_motorInner;
	private final ElevatorIOInputsAutoLogged m_inputsOuter = new ElevatorIOInputsAutoLogged();
	private final ElevatorIOInputsAutoLogged m_inputsInner = new ElevatorIOInputsAutoLogged();

	public Elevator(ElevatorIO ioOuter, ElevatorIO ioInner)
	{
		m_motorOuter = ioOuter;
		m_motorInner = ioInner;
	}

	private void start(double speed, ElevatorState level)
	{
		if (level.getLevel() > 2)
		{
			m_motorOuter.start(speed, ElevatorState.L3);
			if (level.getLevel() == 3)
			{
				m_motorInner.start(speed, ElevatorState.L1);
			} else
			{
				m_motorInner.start(speed, ElevatorState.L2);
			}
		} else
		{
			m_motorOuter.start(speed, ElevatorState.L1);
			if (level.getLevel() == 1)
			{
				m_motorInner.start(speed, ElevatorState.L1);
			} else
			{
				m_motorInner.start(speed, ElevatorState.L2);
			}
		}
	}

	private void stop()
	{
		m_motorOuter.stop();
		m_motorInner.stop();
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
				return m_inputsInner.isAtTargetPosition && m_inputsOuter.isAtTargetPosition;
			}

			@Override
			public void end(boolean isInterrupted)
			{
				stop();
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
		m_motorOuter.updateInputs(m_inputsOuter);
		m_motorInner.updateInputs(m_inputsInner);
		Logger.processInputs(getName() + "/outer", m_inputsOuter);
		Logger.processInputs(getName() + "/inner", m_inputsInner);
	}
}
