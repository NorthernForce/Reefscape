package frc.robot.subsystems.led;

public class LEDIOSim implements LEDIO {
    @Override
    public void updateInputs(LEDIOInputs inputs) {
        inputs.isConnected = true;
    }
    /*
     * These are some simulation methods that do nothing in actual code 
     * but might be helpful for debugging
     */

    @Override
    public void setColor(int r, int g, int b) {
        System.out.println("LED Sim: Setting all LEDs to RGB(" + r + "," + g + "," + b + ")");
    }

    @Override
    public void setLED(int index, int r, int g, int b) {
        System.out.println("LED Sim: Setting LED " + index + " to RGB(" + r + "," + g + "," + b + ")");
    }

    @Override
    public void setLEDRange(int startIndex, int count, int r, int g, int b) {
        System.out.println("LED Sim: Setting LEDs " + startIndex + " to " + (startIndex + count - 1) + 
                         " to RGB(" + r + "," + g + "," + b + ")");
    }

    @Override
    public void clearLEDs() {
        System.out.println("LED Sim: Clearing all LEDs");
    }
}
