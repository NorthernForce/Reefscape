package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase
{
	private final LedsIO io;

	public LEDS(LedsIO io)
	{
		this.io = io;
	}

	@Override
	public void periodic()
	{
		LedsIO.LedIOInputs inputs = new LedsIO.LedIOInputs();
		io.updateInputs(inputs);
	}

	public void setLEDColour(int r, int g, int b)
	{
		io.setColours(r, g, b);
	}

	public void switchLEDs(boolean switched)
	{
		io.setOn(switched);
	}

	public void setLEDBrightness(double brightness)
	{
		io.setBrightness(brightness);
	}

	public Command getSetColour(int r, int g, int b)
	{

		return runOnce(() -> setLEDColour(r, g, b));
	}
}
