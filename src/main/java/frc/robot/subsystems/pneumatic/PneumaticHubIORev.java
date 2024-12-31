package frc.robot.subsystems.pneumatic;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.util.PneumaticConstants;

public class PneumaticHubIORev implements PneumaticHubIO
{
	private final PneumaticConstants m_constants;
	private final Compressor m_compressor;

	public PneumaticHubIORev(PneumaticConstants constants)
	{
		m_constants = constants;
		m_compressor = new Compressor(constants.canId(), PneumaticsModuleType.REVPH);
	}

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

	@Override
	public void enable()
	{
		m_compressor.enableHybrid(m_constants.minPressure(), m_constants.maxPressure());
	}

	@Override
	public void disable()
	{
		m_compressor.disable();
	}

	@Override
	public void updateInputs(PneumaticHubIOInputs inputs)
	{
		inputs.isAtPressure = m_compressor.getPressureSwitchValue();
	}
}