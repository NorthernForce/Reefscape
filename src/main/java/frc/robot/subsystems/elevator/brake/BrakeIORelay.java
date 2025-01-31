package frc.robot.subsystems.elevator.brake;

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
     * @param clockwise true if the relay is clockwise, false if the relay is
     *                  counter clockwise
     */

    public BrakeIORelay(int id, boolean clockwise)
    {
        m_brake = new Relay(id);
        m_brake.setDirection(clockwise ? Relay.Direction.kForward : Relay.Direction.kReverse);
    }

    /**
     * Sets the brake to on or off
     * 
     * @param on true to set the brake on, false to set the brake off
     */

    @Override
    public void setBreak(boolean on)
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
