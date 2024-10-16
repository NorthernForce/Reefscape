package frc.robot.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Dashboard
{
	public void testSend()
	{
		Shuffleboard.getTab("Numbers").add("Pi", 3.14);
		doomLoop();
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