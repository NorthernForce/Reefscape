package frc.robot.subsystems.rollers.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The IO for the rollers beam break sensor.
 */

public class RollersSensorIOBeamBreak implements RollersSensorIO
{
	private final DigitalInput beamBreakSensor;

    /**
     * Constructs a new RollersSensorIOBeamBreak.
     * @param port The port of the beam break sensor.
     */

	public RollersSensorIOBeamBreak(int port)
	{
		beamBreakSensor = new DigitalInput(port);
	}

    /**
     * Updates the inputs for the rollers beam break sensor.
     * @param inputs The inputs to update.
     */

	@Override
	public void updateInputs(RollersSensorIOInputs inputs)
	{
		inputs.hasPiece = !beamBreakSensor.get();
	}

}
