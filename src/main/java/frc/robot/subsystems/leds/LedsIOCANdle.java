package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.CAN;
import frc.robot.subsystems.leds.LedsIO;

public class LedsIOCANdle implements LedsIO
{
	CANdle candle;
	CANdleConfiguration config;

	private int r = 0;
	private int g = 0;
	private int b = 0;

	// Constructor
	public LedsIOCANdle()
	{
		candle = new CANdle(0);
		config = new CANdleConfiguration();
		config.stripType = LEDStripType.RGB;
		config.brightnessScalar = 0.5; // half brightness
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
