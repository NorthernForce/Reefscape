package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
    @AutoLog
    public static class LEDIOInputs {
        public boolean isConnected = false;
    }

    /** updates the set of loggable inputs. */
    public default void updateInputs(LEDIOInputs inputs) {}

    /** sets the led strip to a solid color */
    public default void setColor(int r, int g, int b) {}

    /** sets a specific led to a color */
    public default void setLED(int index, int r, int g, int b) {}

    /** sets a range of leds to a color */
    public default void setLEDRange(int startIndex, int count, int r, int g, int b) {}

    /** clears all leds */
    public default void clearLEDs() {}
}
