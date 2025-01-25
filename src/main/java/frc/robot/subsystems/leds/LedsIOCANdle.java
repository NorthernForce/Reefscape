package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

public class LedsIOCANdle implements LedsIO
{
	// Variables
	CANdle candle;
	CANdleConfiguration config;

	private final int ledCount = 64;

	private int r = 0;
	private int g = 0;
	private int b = 0;
	private boolean animating = false;
	private double speed = 1.0;
	private int animationIndex = 0;

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
		// LedsIO.animating is final and cannot be reassigned
	}

	@Override
	public void setBrightness(double brightness)
	{
		if (brightness >= 0.0 && brightness <= 1.0)
		{
			config.brightnessScalar = brightness;
			candle.configAllSettings(config);
		}
		animating = false;
	}

	@Override
	public void setColours(int rInput, int gInput, int bInput)
	{
		r = rInput;
		g = gInput;
		b = bInput;
		candle.setLEDs(r, g, b);
		animating = false;
	}

	@Override
	public void rainbowAnimation(int ledCount, double speed, double brightness)
	{
		RainbowAnimation rainbowAnim = new RainbowAnimation(brightness, speed, ledCount);
		candle.animate(rainbowAnim);
		animating = true;
	}

	@Override
	public void twinkleAnimation(int r, int g, int b, double speed)
	{
		TwinkleAnimation twinkleAnim = new TwinkleAnimation(r, g, b, 0, speed, ledCount, TwinklePercent.Percent64);
		candle.animate(twinkleAnim);
		animating = true;
	}

	@Override
	public void colourFlow(int r, int g, int b, double speed, boolean direction, int offSet)
	{
		if (direction)
		{
			ColorFlowAnimation colorFlowAnim = new ColorFlowAnimation(r, g, b, 0, speed, ledCount, Direction.Forward,
					offSet);
			candle.animate(colorFlowAnim);
		} else
		{
			ColorFlowAnimation colorFlowAnim = new ColorFlowAnimation(r, g, b, 0, speed, ledCount, Direction.Backward,
					offSet);
			candle.animate(colorFlowAnim);
		}
		animating = true;
	}

	@Override
	public void strobeAnimation(int r, int g, int b, double speed)
	{
		StrobeAnimation strobeAnim = new StrobeAnimation(r, g, b, 0, speed, ledCount);
		candle.animate(strobeAnim);
		animating = true;
	}

	@Override
	public void clearAnimationBuffer()
	{
		for (int i = 0; i < ledCount; i++)
		{
			candle.clearAnimation(i);
		}
		animating = false;
	}

	@Override
	public void incrementAnimation()
	{
		animationIndex++;
		switch (animationIndex)
		{
		case 0:
			setColours(255, 0, 0);
			break;
		case 1:
			setColours(0, 255, 0);
			break;
		case 2:
			setColours(0, 0, 255);
			break;
		case 3:
			setColours(255, 255, 0);
			break;
		case 4:
			setColours(0, 255, 255);
			break;
		case 5:
			setColours(255, 0, 255);
			break;
		case 6:
			setColours(255, 255, 255);
			break;
		case 7:
			rainbowAnimation(ledCount, 0.5, 1.0);
			break;
		case 8:
			twinkleAnimation(255, 0, 0, 0.1);
			break;
		case 9:
			colourFlow(255, 0, 0, 0.1, true, 0);
			break;
		case 10:
			strobeAnimation(255, 0, 0, 0.1);
			break;
		case 11:
			setColours(0, 0, 0);
			clearAnimationBuffer();
		default:
			animationIndex = 0;
			break;
		}
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
		inputs.animating = animating;
		inputs.animationIndex = animationIndex;
	}
}
