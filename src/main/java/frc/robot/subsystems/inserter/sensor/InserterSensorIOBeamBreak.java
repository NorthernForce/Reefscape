package frc.robot.subsystems.inserter.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The IO for the inserter beam break sensor.
 */

public class InserterSensorIOBeamBreak implements InserterSensorIO
{
    private final DigitalInput beamBreakSensor;

    /**
     * Constructs a new InserterSensorIOBeamBreak.
     * 
     * @param port The port of the beam break sensor.
     */

    public InserterSensorIOBeamBreak(int port)
    {
        beamBreakSensor = new DigitalInput(port);
    }

    /**
     * Updates the inputs for the inserter beam break sensor.
     * 
     * @param inputs The inputs to update.
     */

    @Override
    public void updateInputs(InserterSensorIOInputs inputs)
    {
        inputs.hasPiece = !beamBreakSensor.get();
    }

}
