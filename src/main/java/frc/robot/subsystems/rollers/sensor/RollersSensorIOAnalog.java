package frc.robot.subsystems.rollers.sensor;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * The IO for the rollers analog sensor.
 */
public class RollersSensorIOAnalog implements RollersSensorIO
{
    private final AnalogInput m_analogInput;
    private final Distance m_distToObject;

    /**
     * Constructs a new RollersSensorIOAnalog.
     * 
     * @param channel      the channel for the analog sensor (on the roborio ANALOG
     *                     ports)
     * @param distToObject the distance to the object to mark as having a piece
     */
    public RollersSensorIOAnalog(int channel, Distance distToObject)
    {
        m_analogInput = new AnalogInput(channel);
        m_distToObject = distToObject;
    }

    /**
     * Gets the distance from the analog sensor to the object.
     * 
     * @return the distance
     */
    private Distance getDistance()
    {
        return Centimeters.of(m_analogInput.getVoltage() * 5.0 / RobotController.getVoltage5V() * 0.125);
    }

    @Override
    public void updateInputs(RollersSensorIOInputs inputs)
    {
        Logger.recordOutput("Distance" + m_analogInput.getChannel(), getDistance().in(Inches));
        inputs.hasPiece = getDistance().lte(m_distToObject);
    }
}
