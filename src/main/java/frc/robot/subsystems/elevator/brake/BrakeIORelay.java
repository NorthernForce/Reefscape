package frc.robot.subsystems.elevator.brake;

import edu.wpi.first.wpilibj.Relay;

public class BrakeIORelay implements BrakeIO
{
	private Relay m_brake;

	public BrakeIORelay(int index)
	{
		m_brake = new Relay(index);
	}

	@Override
	public void setBreak(boolean on)
	{
		m_brake.set(on ? Relay.Value.kOn : Relay.Value.kOff);
	}

	@Override
	public void setDirection(boolean clockwise)
	{
		m_brake.setDirection(clockwise ? Relay.Direction.kForward : Relay.Direction.kReverse);
	}

	@Override
	public void updateInputs(BrakeIOInputs inputs)
	{
		inputs.isOn = m_brake.get();
	}
}
