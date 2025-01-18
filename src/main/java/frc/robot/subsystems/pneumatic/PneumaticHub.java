package frc.robot.subsystems.pneumatic;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticHub extends SubsystemBase
{

	private final PneumaticHubIO m_io;
	private final PneumaticHubIOInputsAutoLogged m_inputs = new PneumaticHubIOInputsAutoLogged();

	/**
	 * Initialize new PneumaticHub
	 * 
	 * @param io the io instance to pass in
	 */
	public PneumaticHub(PneumaticHubIO io)
	{
		m_io = io;
	}

	/**
	 * toggles whether the pneumatic hub hardware interface is enabled or disabled
	 */
	public void toggle()
	{
		m_io.toggle();
	}

	/**
	 * enables the pneumatic hub io hardware interface
	 */
	public void enable()
	{
		m_io.enable();
	}

	/**
	 * disables the pneumatic hub io hardware interface
	 */
	public void disable()
	{
		m_io.disable();
	}

	/**
	 * periodic input update and processing
	 */

    @Override
	public void periodic()
	{
		m_io.updateInputs(m_inputs);
		Logger.processInputs(getName(), m_inputs);
	}
}