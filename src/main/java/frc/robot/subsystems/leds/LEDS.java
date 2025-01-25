package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



 /**
  * @see frc.robot.subsystems.leds.LEDS
  * @param setLEDColour sets the colour of the LED
    * @param switchLEDs switches the LED on or off
    * @param setLEDBrightness sets the brightness of the LED
    * @param getSetColour gets the colour of the LED
    * @param io sets the LED IO
    * @param periodic updates the LED IO
  * @param r sets the red value of the LED
  * @param g sets the green value of the LED
  * @param b sets the blue value of the LED
  */

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
