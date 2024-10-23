package frc.robot.robots.subsystem;

import frc.robot.Constants;
import frc.robot.robots.subsystem.CANdleSystem;
import edu.wpi.first.wpilibj.util.Color;

public class CANdleSystem
{
	private CANdleSystem m_candleSystem;

	public CANdleSystem(int candleid)
	{

	}

	public CANdleSystem()
	{
		// Initialize the CANdleSystem with the CANdle ID from Constants
		m_candleSystem = new CANdleSystem(Constants.CANdleID);
	}

	public void setLEDColor()
	{
		// Set the desired color (e.g., Red)
		Color redColor = new Color(255, 0, 0); // RGB value for Red
		m_candleSystem.setColor(redColor); // Set the color of the LEDs
	}

	private void setColor(Color redColor)
	{

		throw new UnsupportedOperationException("Unimplemented method 'setColor'");
	}

	// Other methods and logic for your robot...
}