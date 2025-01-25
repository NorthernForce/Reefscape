package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.rollers.sensor.SensorIOInputsAutoLogged;
import frc.robot.subsystems.rollers.sensor.SensorIO;

public class Rollers implements Subsystem
{
	public RollersIO intakeIO;
	public SensorIO sensorIO;
	private IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();
	private SensorIOInputsAutoLogged m_beamBreakInputs = new SensorIOInputsAutoLogged();

	public Rollers(RollersIO intakeIO, SensorIO sensorIO)
	{
		this.intakeIO = intakeIO;
		this.sensorIO = sensorIO;
	}

	public void intake()
	{
		intakeIO.set(1);
	}

	public void outtake()
	{
		intakeIO.set(-1);
	}

	public void stop()
	{
		intakeIO.set(0);
	}

	public Command getIntakeCommand()
	{
		return new Command()
		{
			@Override
			public void execute()
			{
				intake();
			}

			@Override
			public boolean isFinished()
			{
				return m_beamBreakInputs.hasPiece;
			}

			@Override
			public void end(boolean interrupted)
			{
				stop();
			}
		};
	}

	public void periodic()
	{
		intakeIO.updateInputs(m_inputs);
		sensorIO.updateInputs(m_beamBreakInputs);
		Logger.processInputs(getName() + "/Motor", m_inputs);
		Logger.processInputs(getName() + "/BeamBreak", m_beamBreakInputs);
	}
}
