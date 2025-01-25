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

	/**
	 * Initializes the CANdle for leds
	 * 
	 * @param id id of the CANdle on the rio
	 */
	// Constructor
	public LedsIOCANdle(int id)
	{
		candle = new CANdle(id);
		config = new CANdleConfiguration();
		config.stripType = LEDStripType.RGB;
		config.brightnessScalar = 1.0;
		candle.configAllSettings(config);
	}

	/**
	 * turns the leds on or off
	 * 
	 * @param on boolean for on or off true: on false: off
	 */
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

	/**
	 * sets the brightness of the leds
	 * 
	 * @param brightness double of 0.0 to 1.0 scale
	 */
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

	/**
	 * sets the colors of all of the leds to r, g, b
	 * 
	 * @param rInput red colour of the leds
	 * @param gInput green colour of the leds
	 * @param bInput blue colour of the leds
	 */
	@Override
	public void setColours(int rInput, int gInput, int bInput)
	{
		r = rInput;
		g = gInput;
		b = bInput;
		candle.setLEDs(r, g, b);
		animating = false;
	}

	/**
	 * Uses CANdles built in rainbow animation to create a rainbow effect for the
	 * leds
	 * 
	 * @param ledCount   the amount of leds this takes effect upon
	 * @param speed      a double 0.0 to 1.0 determining how fast the animation
	 *                   happens
	 * @param brightness a double 0.0 to 1.0 determining how bright the leds are
	 */
	@Override
	public void rainbowAnimation(int ledCount, double speed, double brightness)
	{
		RainbowAnimation rainbowAnim = new RainbowAnimation(brightness, speed, ledCount);
		candle.animate(rainbowAnim);
		animating = true;
	}

	/**
	 * Uses CANdles built in twinkle animation to create a twinkle effect for the
	 * leds
	 * 
	 * @param r     sets the red colour of the leds
	 * @param g     sets the green colour of the leds
	 * @param b     sets the blue colour of the leds
	 * @param speed a double 0.0 to 1.0 that determines the speed of the animation
	 */
	@Override
	public void twinkleAnimation(int r, int g, int b, double speed)
	{
		TwinkleAnimation twinkleAnim = new TwinkleAnimation(r, g, b, 0, speed, ledCount, TwinklePercent.Percent64);
		candle.animate(twinkleAnim);
		animating = true;
	}

	/**
	 * Uses CANdle colourflow animation to create a flow effect for the leds
	 * 
	 * @param r         sets the red colour of the leds
	 * @param g         sets the green colour of the leds
	 * @param b         sets the blue colour of the leds
	 * @param speed     sets the speed of the flow effect 0.0 to 1.0
	 * @param direction boolean true: flows forwards false: flows backwards
	 * @param offset    offset of the flow animation
	 */
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

	/**
	 * uses CANdle srobe animation to create a strobe effect with the leds
	 * 
	 * @param r sets the red colour of the leds
	 * @param g sets the green colour of the leds
	 * @param b sets the blue colour of the leds
	 * @speed double 0.0 to 1.0 controls the speed of the strobe
	 */
	@Override
	public void strobeAnimation(int r, int g, int b, double speed)
	{
		StrobeAnimation strobeAnim = new StrobeAnimation(r, g, b, 0, speed, ledCount);
		candle.animate(strobeAnim);
		animating = true;
	}

	/**
	 * clears all of the animation channels to stop any animations
	 */
	@Override
	public void clearAnimationBuffer()
	{
		for (int i = 0; i < ledCount; i++)
		{
			candle.clearAnimation(i);
		}
		animating = false;
	}

	/**
	 * changes the current state of the ledsto the next one so they can be cycled
	 * through automaticaly
	 */
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
