package frc.robot.subsystems.superstructure.elevator.brake;

import edu.wpi.first.wpilibj.Relay;

/**
 * BrakeIORelay is a class that implements BrakeIO using a relay.
 */

public class BrakeIORelay implements BrakeIO
{
    private Relay m_brake;

    /**
     * Creates a new BrakeIORelay
     * 
     * @param id        the id of the relay
     */

    public BrakeIORelay(int id)
    {
        m_brake = new Relay(id);
        m_brake.setDirection(Relay.Direction.kForward);
    }

    /**
     * Sets the brake to on or off
     * 
     * @param on true to set the brake on, false to set the brake off
     */

    @Override
    public void setBrake(boolean on)
    {
        m_brake.set(on ? Relay.Value.kOn : Relay.Value.kOff);
    }

    /**
     * Updates the inputs for the brake
     * 
     * @param inputs the inputs to update
     */

    @Override
    public void updateInputs(BrakeIOInputs inputs)
    {
        inputs.isOn = m_brake.get() != Relay.Value.kOff;
    }
}
