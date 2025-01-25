package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.elevator.brake.BrakeIO;
import frc.robot.subsystems.elevator.brake.BrakeIOInputsAutoLogged;
import frc.robot.zippy.constants.ZippyConstants.ElevatorConstants.ElevatorState;

public class Elevator implements Subsystem
{
	private ElevatorIO m_motorOuter;
	private ElevatorIO m_motorInner;
	private final ElevatorIOInputsAutoLogged m_inputsOuter = new ElevatorIOInputsAutoLogged();
	private final ElevatorIOInputsAutoLogged m_inputsInner = new ElevatorIOInputsAutoLogged();
	private final BrakeIOInputsAutoLogged m_brakeInputsOuter = new BrakeIOInputsAutoLogged();
	private final BrakeIOInputsAutoLogged m_brakeInputsInner = new BrakeIOInputsAutoLogged();
	private BrakeIO m_brakeOuter;
	private BrakeIO m_brakeInner;

	public Elevator(ElevatorIO ioOuter, ElevatorIO ioInner, BrakeIO breakOuter, BrakeIO breakInner)
	{
		m_motorOuter = ioOuter;
		m_motorInner = ioInner;
		m_brakeOuter = breakOuter;
		m_brakeInner = breakInner;
	}

	private void setTargetPosition(double speed, ElevatorState level)
	{
		m_brakeInner.setBreak(false);
		m_brakeOuter.setBreak(false);
		m_motorOuter.setTargetPosition(speed, level.getOuterHeight());
		m_motorInner.setTargetPosition(speed, level.getInnerHeight());
	}

	private void stop()
	{
		m_motorOuter.stop();
		m_motorInner.stop();
		m_brakeInner.setBreak(true);
		m_brakeOuter.setBreak(true);
	}

	public Command getLevelCommand(ElevatorState level)
	{

		return new Command()
		{
			@Override
			public void initialize()
			{
				setTargetPosition(0.5, level);
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
		m_brakeOuter.updateInputs(m_brakeInputsOuter);
		m_brakeInner.updateInputs(m_brakeInputsInner);
		Logger.processInputs(getName() + "/outer", m_inputsOuter);
		Logger.processInputs(getName() + "/inner", m_inputsInner);
		Logger.processInputs(getName() + "/brake/outer", m_brakeInputsOuter);
		Logger.processInputs(getName() + "/brake/inner", m_brakeInputsInner);
	}
}
