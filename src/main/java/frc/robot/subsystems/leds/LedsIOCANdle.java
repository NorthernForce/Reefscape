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

import edu.wpi.first.units.measure.Angle;

public class LedsIOCANdle implements LedsIO
{
    // Variables
    private CANdle candle;
    private CANdleConfiguration config;

    /**
     * Initializes the CANdle for leds
     * 
     * @param id          id of the CANdle on the rio
     * @param ledSettings the settings for the leds
     */
    // Constructor

    private int r = 0;
    private int g = 0;
    private int b = 0;
    private boolean on = true;
    private int ledCount = 0;
    private double brightness = 0;
    private double animationSpeed = 0;
    private boolean animating = true;
    private int animationIndex = 0;

    public LedsIOCANdle(int id, LedConstantsRecord ledSettings)
    {
        initCANdle(id);
        ledCount = ledSettings.ledCount();
        brightness = ledSettings.ledBrightness();
        animationSpeed = ledSettings.animationSpeed();
        animating = ledSettings.animating();
        animationIndex = ledSettings.animationIndex();
    }

    public void initCANdle(int id)
    {
        candle = new CANdle(id);
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = brightness;
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
        this.on = on;
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
            this.brightness = brightness;
            config.brightnessScalar = brightness;
            candle.configAllSettings(config);
        }
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
        candle.setLEDs(rInput, gInput, bInput);
        animating = false;
    }

    /**
     * untested but should be able to set a range of leds to a specific colour
     * 
     * @param startIdx the start index of the leds
     * @param endIdx   the end index of the leds
     * @param r        the red value of the leds
     * @param g        the green value of the leds
     * @param b        the blue value of the leds
     */
    @Override
    public void setSpecificLEDs(int startIdx, int endIdx, int r, int g, int b)
    {
        this.r = r;
        this.g = g;
        this.b = b;
        candle.setLEDs(r, g, b, 0, startIdx, Math.abs(endIdx - startIdx));
        animating = false;
    }

    /**
     * Uses CANdles built in rainbow animation to create a rainbow effect for the
     * leds
     */
    @Override
    public void rainbowAnimation()
    {
        RainbowAnimation rainbowAnim = new RainbowAnimation(brightness, animationSpeed, ledCount);
        candle.animate(rainbowAnim);
        animating = true;
    }

    /**
     * Uses CANdles built in twinkle animation to create a twinkle effect for the
     * leds
     * 
     * @param r sets the red colour of the leds
     * @param g sets the green colour of the leds
     * @param b sets the blue colour of the leds
     */
    @Override
    public void twinkleAnimation(int r, int g, int b)
    {
        this.r = r;
        this.g = g;
        this.b = b;
        TwinkleAnimation twinkleAnim = new TwinkleAnimation(r, g, b, 0, animationSpeed, ledCount,
                TwinklePercent.Percent64);
        candle.animate(twinkleAnim);
        animating = true;
    }

    /**
     * Uses CANdle colourflow animation to create a flow effect for the leds
     * 
     * @param r         sets the red colour of the leds
     * @param g         sets the green colour of the leds
     * @param b         sets the blue colour of the leds
     * @param direction boolean true: flows forwards false: flows backwards
     * @param offset    offset of the flow animation
     */
    @Override
    public void colourFlow(int r, int g, int b, boolean direction, int offSet)
    {
        this.r = r;
        this.g = g;
        this.b = b;
        if (direction)
        {
            ColorFlowAnimation colorFlowAnim = new ColorFlowAnimation(r, g, b, 0, animationSpeed, ledCount,
                    Direction.Forward, offSet);
            candle.animate(colorFlowAnim);
        } else
        {
            ColorFlowAnimation colorFlowAnim = new ColorFlowAnimation(r, g, b, 0, animationSpeed, ledCount,
                    Direction.Backward, offSet);
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
     */
    @Override
    public void strobeAnimation(int r, int g, int b)
    {
        this.r = r;
        this.g = g;
        this.b = b;
        StrobeAnimation strobeAnim = new StrobeAnimation(r, g, b, 0, animationSpeed, ledCount);
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
    }

    @Override
    public void compassEffect(Angle degrees)
    {
        // TODO Auto-generated method stub
    }

    /**
     * changes the current state of the ledsto the next one so they can be cycled
     * through automaticaly
     */
    @Override
    public void incrementAnimation()
    {
        if (animating)
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
                rainbowAnimation();
                break;
            case 8:
                twinkleAnimation(255, 0, 0);
                break;
            case 9:
                colourFlow(255, 0, 0, true, 0);
                break;
            case 10:
                strobeAnimation(255, 0, 0);
                break;
            case 11:
                setColours(0, 0, 0);
                clearAnimationBuffer();
            case 12:
                setSpecificLEDs(0, 10, 255, 0, 0);
                break;
            default:
                break;
            }
        }
    }

    public void lightList(int[] leds, int r, int g, int b)
    {
        for (int i = 0; i < leds.length; i++)
        {
            candle.setLEDs(r, g, b,0, leds[i], 1);
        }
    }

    @Override
    public void updateInputs(LedIOInputs inputs)
    {
        candle.getAllConfigs(config);
        inputs.r = r;
        inputs.g = g;
        inputs.b = b;
        inputs.on = on;
        inputs.ledCount = ledCount;
        inputs.brightness = brightness;
        inputs.animating = animating;
        inputs.animationIndex = animationIndex;
    }
}
