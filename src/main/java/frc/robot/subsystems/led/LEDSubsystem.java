package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
    private final LEDIO io;
    private final LEDIOInputs inputs = new LEDIOInputs();

    private static final int CORAL_STATUS_START = 0;
    private static final int CORAL_STATUS_LENGTH = 8;
    private static final int REEF_READY_START = 8;
    private static final int REEF_READY_LENGTH = 8;
    private static final int FAULT_STATUS_START = 16;
    private static final int FAULT_STATUS_LENGTH = 8;
    private static final int AUTO_STATUS_START = 24;
    private static final int AUTO_STATUS_LENGTH = 8;

    private static final class Colors {
        static final int[] GREEN = {0, 255, 0};
        static final int[] RED = {255, 0, 0};
        static final int[] YELLOW = {255, 255, 0};
        static final int[] BLUE = {0, 0, 255};
        static final int[] OFF = {0, 0, 0};
    }

    private boolean hasCoral = false;
    private boolean isReefReady = false;
    private boolean hasFault = false;
    private boolean isAutoSelected = false;

    public LEDSubsystem(LEDIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LED", inputs);
        updateLEDs();
    }

    private void updateLEDs() {
        if (!inputs.isConnected) {
            return;
        }
// wow im so cool with my ternary operators right
        setSection(CORAL_STATUS_START, CORAL_STATUS_LENGTH, hasCoral ? Colors.GREEN : Colors.OFF);
        setSection(REEF_READY_START, REEF_READY_LENGTH, isReefReady ? Colors.BLUE : Colors.OFF);
        setSection(FAULT_STATUS_START, FAULT_STATUS_LENGTH, hasFault ? Colors.RED : Colors.OFF);
        setSection(AUTO_STATUS_START, AUTO_STATUS_LENGTH, isAutoSelected ? Colors.YELLOW : Colors.OFF);
    }

    private void setSection(int start, int length, int[] color) {
        io.setLEDRange(start, length, color[0], color[1], color[2]);
    }

    public void setHasCoral(boolean hasCoral) {
        this.hasCoral = hasCoral;
    }

    public void setReefReady(boolean isReefReady) {
        this.isReefReady = isReefReady;
    }

    public void setFault(boolean hasFault) {
        this.hasFault = hasFault;
    }

    public void setAutoSelected(boolean isAutoSelected) {
        this.isAutoSelected = isAutoSelected;
    }
}
