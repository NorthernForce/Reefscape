package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.rollers.sensor.SensorIOInputsAutoLogged;
import frc.robot.subsystems.rollers.sensor.SensorIO;

public class Rollers implements Subsystem
{
	public RollersIO m_intakeIO;
	public SensorIO m_sensorIOAlgae;
	public SensorIO m_sensorIOCoral;
	private IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();
	private SensorIOInputsAutoLogged m_sensorIOAlgaeInputs = new SensorIOInputsAutoLogged();
	private SensorIOInputsAutoLogged m_sensorIOCoralInputs = new SensorIOInputsAutoLogged();

	public Rollers(RollersIO intakeIO, SensorIO sensorIOAlgae, SensorIO sensorIOCoral)
	{
		m_intakeIO = intakeIO;
		m_sensorIOAlgae = sensorIOAlgae;
		m_sensorIOCoral = sensorIOCoral;
	}

	public void intake()
	{
		m_intakeIO.set(1);
	}

	public void outtake()
	{
		m_intakeIO.set(-1);
	}

	public void stop()
	{
		m_intakeIO.set(0);
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
				return m_sensorIOAlgaeInputs.hasPiece || m_sensorIOCoralInputs.hasPiece;
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
		m_intakeIO.updateInputs(m_inputs);
		m_sensorIOAlgae.updateInputs(m_sensorIOAlgaeInputs);
		m_sensorIOCoral.updateInputs(m_sensorIOCoralInputs);
		Logger.processInputs(getName() + "/Motor", m_inputs);
		Logger.processInputs(getName() + "/sensors/algae", m_sensorIOAlgaeInputs);
		Logger.processInputs(getName() + "/sensors/coral", m_sensorIOCoralInputs);
	}
}
