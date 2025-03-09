package frc.robot.subsystems.inserter.sensor;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Ultrasonic;

import static edu.wpi.first.units.Units.Millimeters;

/**
 * The IO for the inserter ultrasonic sensor.
 */

public class InserterSensorIOUltrasonic implements InserterSensorIO
{
    private final Ultrasonic m_sonar;
    private final Distance m_distToObject;

    /**
     * Constructs a new InserterSensorIOUltrasonic.
     * 
     * @param trigChannel the channel for the trigger pin
     * @param echoChannel the channel for the echo pin (receiving end)
     * @param mmToObject  the distance in mm to the object to mark as having a piece
     */

    public InserterSensorIOUltrasonic(int trigChannel, int echoChannel, Distance distToObject)
    {
        m_sonar = new Ultrasonic(trigChannel, echoChannel);
        m_distToObject = distToObject;
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
     * Updates the inputs for the inserter ultrasonic sensor.
     * 
     * @param inputs The inputs to update.
     */

    @Override
    public void updateInputs(InserterSensorIOInputs inputs)
    {
        inputs.hasPiece = getRange() <= m_distToObject.in(Millimeters);
    }

}
