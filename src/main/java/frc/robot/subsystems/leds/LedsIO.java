package frc.robot.subsystems.leds;

public interface LedsIO
{
    public record LedConstantsRecord(int ledCount, double ledBrightness, double animationSpeed, boolean animating,
            int animationIndex) {
    }

    static class LedIOInputs
    {
        public LedConstantsRecord ledIOSettings;

        public LedIOInputs(LedConstantsRecord ledIOSettings)
        {
            this.ledIOSettings = ledIOSettings;
        }

        public LedIOInputs()
        {
        }

        public void setLedCount(int ledCount)
        {
            ledIOSettings = new LedConstantsRecord(ledCount, ledIOSettings.ledBrightness(),
                    ledIOSettings.animationSpeed(), ledIOSettings.animating(), ledIOSettings.animationIndex());
        }

        public void setLedBrightness(double ledBrightness)
        {
            ledIOSettings = new LedConstantsRecord(ledIOSettings.ledCount(), ledBrightness,
                    ledIOSettings.animationSpeed(), ledIOSettings.animating(), ledIOSettings.animationIndex());
        }

        public void setAnimationSpeed(double animationSpeed)
        {
            ledIOSettings = new LedConstantsRecord(ledIOSettings.ledCount(), ledIOSettings.ledBrightness(),
                    animationSpeed, ledIOSettings.animating(), ledIOSettings.animationIndex());
        }

        public void setAnimating(boolean animating)
        {
            ledIOSettings = new LedConstantsRecord(ledIOSettings.ledCount(), ledIOSettings.ledBrightness(),
                    ledIOSettings.animationSpeed(), animating, ledIOSettings.animationIndex());
        }

        public void setAnimationIndex(int animationIndex)
        {
            ledIOSettings = new LedConstantsRecord(ledIOSettings.ledCount(), ledIOSettings.ledBrightness(),
                    ledIOSettings.animationSpeed(), ledIOSettings.animating(), animationIndex);
        }

        public void setLedIOSettings(LedConstantsRecord ledIOSettings)
        {
            this.ledIOSettings = ledIOSettings;
        }

        public LedConstantsRecord getLedIOSettings()
        {
            return ledIOSettings;
        }
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

    public default void incrementAnimation()
    {
    }

    public default void clearAnimationBuffer()
    {
    }
}
