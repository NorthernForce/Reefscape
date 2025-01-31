package frc.robot.subsystems.rollers.sensor;

import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * The IO for the rollers ultrasonic sensor.
 */

public class RollersSensorIOUltrasonic implements RollersSensorIO
{
    private final Ultrasonic m_sonar;
    private final double m_mmToObject;

    /**
     * Constructs a new RollersSensorIOUltrasonic.
     * 
     * @param trigChannel the channel for the trigger pin
     * @param echoChannel the channel for the echo pin (receiving end)
     * @param mmToObject  the distance in mm to the object to mark as having a piece
     */

    public RollersSensorIOUltrasonic(int trigChannel, int echoChannel, double mmToObject)
    {
        m_sonar = new Ultrasonic(trigChannel, echoChannel);
        m_mmToObject = mmToObject;
    }

    /**
     * Gets the range in mm from the ultrasonic sensor to the object.
     * 
     * @return the range in mm
     */

    private double getRange()
    {
        return m_sonar.getRangeMM();
    }

    /**
     * Updates the inputs for the rollers ultrasonic sensor.
     * 
     * @param inputs The inputs to update.
     */

    @Override
    public void updateInputs(RollersSensorIOInputs inputs)
    {
        inputs.hasPiece = getRange() <= m_mmToObject;
    }

}
