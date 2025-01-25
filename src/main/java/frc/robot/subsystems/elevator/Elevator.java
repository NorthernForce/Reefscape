package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.zippy.constants.ZippyConstants.ElevatorConstants.ElevatorState;

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

	private void start(double speed, ElevatorState levelOuter, ElevatorState levelInner)
	{
		m_motorOuter.setTargetPosition(speed, levelOuter);
		m_motorInner.setTargetPosition(speed, levelInner);
	}

	private void stop()
	{
		m_motorOuter.stop();
		m_motorInner.stop();
	}

	public Command getLevelCommand(ElevatorState levelOuter, ElevatorState levelInner)
	{

		return new Command()
		{
			@Override
			public void initialize()
			{
				start(0.5, levelOuter, levelInner);
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
		return getLevelCommand(ElevatorState.L1Outer, ElevatorState.L1Inner);
	}

	public Command getL2Command()
	{
		return getLevelCommand(ElevatorState.L2Outer, ElevatorState.L2Inner);
	}

	public Command getL3Command()
	{
		return getLevelCommand(ElevatorState.L3Outer, ElevatorState.L3Inner);
	}

	public Command getL4Command()
	{
		return getLevelCommand(ElevatorState.L4Outer, ElevatorState.L4Inner);
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
