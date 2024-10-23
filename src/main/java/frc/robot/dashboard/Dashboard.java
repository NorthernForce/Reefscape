package frc.robot.dashboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Dashboard
{
	public void testSend() throws InterruptedException
	{
		double pi = 3.14;
		GenericEntry value = sendString("testing", "name", "Reefscape");
		sendInt("testing", "everett IQ", 115);
		sendFloat("testing", "pi", (float) pi);
		sendDouble("testing", "e", 2.71);
		sendBool("testing", "is everett zesty", true);
		Thread.sleep(10000);
		System.out.println(value.getString("Everett3"));
	}

	public GenericEntry sendString(String tab, String title, String value)
	{
		return Shuffleboard.getTab(tab).add(title, value).getEntry();
	}

	public GenericEntry sendInt(String tab, String title, int value)
	{
		return Shuffleboard.getTab(tab).add(title, value).getEntry();
	}

	public GenericEntry sendDouble(String tab, String title, double value)
	{
		return Shuffleboard.getTab(tab).add(title, value).getEntry();
	}

	public GenericEntry sendFloat(String tab, String title, float value)
	{
		return Shuffleboard.getTab(tab).add(title, value).getEntry();
	}

	public GenericEntry sendBool(String tab, String title, boolean value)
	{
		return Shuffleboard.getTab(tab).add(title, value).getEntry();
	}

	public GenericEntry sendShort(String tab, String title, short value)
	{
		return Shuffleboard.getTab(tab).add(title, value).getEntry();
	}
}