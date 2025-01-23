package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem
{
	public IntakeIO intakeIO;
	private IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

	public Intake(IntakeIO intakeIO)
	{
		this.intakeIO = intakeIO;
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

	public void periodic()
	{
		intakeIO.updateInputs(m_inputs);
		Logger.processInputs(getName(), m_inputs);
	}
}
