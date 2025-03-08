package frc.robot.subsystems.algaeremover.sensor;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeSensorIO {
    @AutoLog
    public static class AlgaeSensorInputs {
        // make sure algae arm is at the top
        public boolean reachedTop = true;
    }

    public default void updateInputs(AlgaeSensorInputs inputs) {
    }
}
