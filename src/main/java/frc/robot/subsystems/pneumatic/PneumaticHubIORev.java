package frc.robot.subsystems.pneumatic;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.util.PneumaticConstants;

/**
 * hardware io layer for the pneumatic hub
 */
public class PneumaticHubIORev implements PneumaticHubIO
{
	private final PneumaticConstants m_constants;
	private final Compressor m_compressor;

	/**
	 * creates new instance of a rev hub hardware io layer
	 */
	public PneumaticHubIORev(PneumaticConstants constants)
	{
		m_constants = constants;
		m_compressor = new Compressor(constants.canId(), PneumaticsModuleType.REVPH);
	}

	/**
	 * toggles on hardware level
	 */
	@Override
	public void toggle()
	{
		if (m_compressor.isEnabled())
		{
			m_compressor.disable();
		} else
		{
			m_compressor.enableHybrid(m_constants.minPressure(), m_constants.maxPressure());
		}
	}

	/**
	 * enables on hardware layer
	 */
	@Override
	public void enable()
	{
		m_compressor.enableHybrid(m_constants.minPressure(), m_constants.maxPressure());
	}

	/**
	 * disables on hardware layer
	 */
	@Override
	public void disable()
	{
		m_compressor.disable();
	}

	/**
	 * gets the name of the module
	 */
	@Override
	public String getName()
	{
		return "Pneumatics/PneumaticHubRev";
	}

	/**
	 * updates the inputs (like if it is at pressure)
	 */
	@Override
	public void updateInputs(PneumaticHubIOInputs inputs)
	{
		inputs.isAtPressure = m_compressor.getPressureSwitchValue();
	}
}