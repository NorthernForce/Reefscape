package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import frc.robot.subsystems.leds.LedsIO;

/**
 * @see frc.robot.subsystems.leds.LedsIOCANdle
 * @param setOn         sets the LED on or off
 * @param setBrightness sets the brightness of the LED
 * @param setColours    sets the colours of the LED
 * @param updateInputs  updates the LED inputs
 * @param r             red value of the LED
 * @param g             green value of the LED
 * @param b             blue value of the LED
 */

public class LedsIOCANdle implements LedsIO
{
	// Variables
	CANdle candle;
	CANdleConfiguration config;

	private int r = 0;
	private int g = 0;
	private int b = 0;

	// Constructor
	public LedsIOCANdle(int id)
	{
		candle = new CANdle(id);
		config = new CANdleConfiguration();
		config.stripType = LEDStripType.RGB;
		config.brightnessScalar = 1.0;
		candle.configAllSettings(config);
	}

	@Override
	public void setOn(boolean on)
	{
		if (!on)
		{
			config.brightnessScalar = 0.0;
			candle.configAllSettings(config);
		}
	}

	@Override
	public void setBrightness(double brightness)
	{
		if (brightness >= 0.0 && brightness <= 1.0)
		{
			config.brightnessScalar = brightness;
			candle.configAllSettings(config);
		}
	}

	@Override
	public void setColours(int rInput, int gInput, int bInput)
	{
		r = rInput;
		g = gInput;
		b = bInput;
		candle.setLEDs(r, g, b);
	}

	@Override
	public void RainbowAnimation()
	{
		Animation anim = new RainbowAnimation();
		candle.animate(anim);
	}

	@Override
	public void updateInputs(LedIOInputs inputs)
	{
		candle.getAllConfigs(config);
		inputs.brightness = config.brightnessScalar;
		if (config.brightnessScalar > 0.0)
		{
			inputs.on = true;
		} else
		{
			inputs.on = false;
		}
		inputs.r = r;
		inputs.g = g;
		inputs.b = b;
	}
}
