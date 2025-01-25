package frc.robot.subsystems.rollers.sensor;

import edu.wpi.first.wpilibj.AnalogInput;

public class SensorIOUltrasonic implements SensorIO
{

	private final AnalogInput ultrasonicSensor;
	private final double distanceToStopAt;

	public SensorIOUltrasonic(int signalChannel, double distanceToStopAt)
	{
		ultrasonicSensor = new AnalogInput(signalChannel);
		this.distanceToStopAt = distanceToStopAt;
	}

	/**
	 * gets the distance from the ultrasonic sensor
	 * 
	 * @return the distance, where 0V = 20 mm, and 5V = 4000 mm. Do the math.
	 */
	private double getDistance()
	{
		return ultrasonicSensor.getVoltage() / 5.0 * 3980.0 + 20.0;
	}

	@Override
	public void updateInputs(SensorIOInputs inputs)
	{
		inputs.hasPiece = getDistance() <= distanceToStopAt;
	}

}
