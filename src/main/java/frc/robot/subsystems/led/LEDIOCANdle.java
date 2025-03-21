package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LEDIOCANdle implements LEDIO {
    private final CANdle candle;
    private static final int LED_COUNT = 32; // idk what the led length is so this is a placeholder

    public LEDIOCANdle(int candleID) {
        candle = new CANdle(candleID);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = com.ctre.phoenix.led.CANdle.LEDStripType.RGB; // this could be wrong
        config.brightnessScalar = 0.5; // brightness bc saving battery is fun
        candle.configAllSettings(config);
    }

    @Override
    public void updateInputs(LEDIOInputs inputs) {
        inputs.isConnected = candle.getFirmwareVersion() != 0;
    }

    @Override
    public void setColor(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    @Override
    public void setLED(int index, int r, int g, int b) {
        if (index >= 0 && index < LED_COUNT) {
            candle.setLEDs(r, g, b, 0, index, 1);
        }
    }

    @Override
    public void setLEDRange(int startIndex, int count, int r, int g, int b) {
        if (startIndex >= 0 && startIndex + count <= LED_COUNT) {
            candle.setLEDs(r, g, b, 0, startIndex, count);
        }
    }

    @Override
    public void clearLEDs() {
        candle.setLEDs(0, 0, 0);
    }
}
