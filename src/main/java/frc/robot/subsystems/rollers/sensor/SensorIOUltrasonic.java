package frc.robot.subsystems.rollers.sensor;

import edu.wpi.first.wpilibj.Ultrasonic;

public class SensorIOUltrasonic implements SensorIO
{
	private final Ultrasonic m_sonar;
	private final double m_mmToObject;

	public SensorIOUltrasonic(int trigChannel, int echoChannel, double mmToObject)
	{
		m_sonar = new Ultrasonic(trigChannel, echoChannel);
		m_mmToObject = mmToObject;
	}

	private double getRange()
	{
		return m_sonar.getRangeMM();
	}

	@Override
	public void updateInputs(SensorIOInputs inputs)
	{
		inputs.hasPiece = getRange() <= m_mmToObject;
	}

}
