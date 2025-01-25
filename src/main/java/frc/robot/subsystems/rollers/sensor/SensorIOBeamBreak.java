package frc.robot.subsystems.rollers.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class SensorIOBeamBreak implements SensorIO
{
	private final DigitalInput beamBreakSensor;

	public SensorIOBeamBreak(int port)
	{
		beamBreakSensor = new DigitalInput(port);
	}

	@Override
	public void updateInputs(SensorIOInputs inputs)
	{
		inputs.hasPiece = !beamBreakSensor.get();
	}

}
