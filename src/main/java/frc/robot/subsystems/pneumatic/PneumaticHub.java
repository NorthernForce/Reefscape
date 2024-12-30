package frc.robot.subsystems.pneumatic;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PneumaticConstants;

public class PneumaticHub extends SubsystemBase
{
	private final PneumaticHubIO m_io;
	private final PneumaticHubIOInputsAutoLogged m_inputs = new PneumaticHubIOInputsAutoLogged();

	public PneumaticHub(PneumaticConstants constants, PneumaticHubIO io)
	{
		m_io = io;
	}

	public void toggle()
	{
		m_io.toggle();
	}

	public void periodic()
	{
		m_io.updateInputs(m_inputs);
		Logger.processInputs("Pneumatics/PneumaticHub", m_inputs);
	}
}