package frc.robot.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Dashboard
{
	public void testSend()
	{
		sendString("testing", "name", "Reefscape");
		sendInt("testing", "everett IQ", 115);
		sendFloat("testing", "pi", 3.14f);
		sendDouble("testing", "e", 2.71);
		sendBool("testing", "is everett zesty", true);
		doomLoop();
	}

	public void sendString(String tab, String title, String value)
	{
		Shuffleboard.getTab(tab).add(title, value);
	}

	public void sendInt(String tab, String title, int value)
	{
		Shuffleboard.getTab(tab).add(title, value);
	}

	public void sendDouble(String tab, String title, double value)
	{
		Shuffleboard.getTab(tab).add(title, value);
	}

	public void sendFloat(String tab, String title, float value)
	{
		Shuffleboard.getTab(tab).add(title, value);
	}

	public void sendBool(String tab, String title, boolean value)
	{
		Shuffleboard.getTab(tab).add(title, value);
	}

	public void sendShort(String tab, String title, short value)
	{
		Shuffleboard.getTab(tab).add(title, value);
	}

	// so it will remain connected
	public void doomLoop()
	{
		while (true)
		{
			try
			{
				Thread.sleep(200);
			} catch (InterruptedException e)
			{
				Thread.currentThread().interrupt();
			}
		}
	}
}