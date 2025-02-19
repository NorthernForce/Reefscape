package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface LedsIO
{
    public record LedConstantsRecord(int ledCount, double ledBrightness, double animationSpeed, boolean animating,
            int animationIndex) {
    }

    @AutoLog
    public static class LedIOInputs
    {
        public int r = 0;
        public int g = 0;
        public int b = 0;
        public boolean on = true;
        public int ledCount = 0;
        public double brightness = 0;
        public boolean animating = true;
        public int animationIndex = -1;
    }

    public default void setColours(int r, int g, int b)
    {
    }

    public default void setOn(boolean on)
    {
    }

    public default void setBrightness(double brightness)
    {
    }

    public default void updateInputs(LedIOInputs inputs)
    {
    }

    public default void updateAnimating(boolean on)
    {
    }

    public default void rainbowAnimation()
    {
    }

    public default void twinkleAnimation(int r, int g, int b)
    {
    }

    public default void colourFlow(int r, int g, int b, boolean direction, int offSet)
    {
    }

    public default void strobeAnimation(int r, int g, int b)
    {
    }

    public default void setSpecificLEDs(int startIdx, int endIdx, int r, int g, int b)
    {
    }

    public default void compassEffect(Angle degrees)
    {
    }

    public default void incrementAnimation()
    {
    }

    public default void clearAnimationBuffer()
    {
    }

    public default void lightList(int[] leds, int r, int g, int b)
    {
    }
}
