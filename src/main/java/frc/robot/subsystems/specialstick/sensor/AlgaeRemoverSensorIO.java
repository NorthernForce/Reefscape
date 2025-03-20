package frc.robot.subsystems.specialstick.sensor;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRemoverSensorIO
{
    @AutoLog
    public static class AlgaeRemoverSensorIOInputs
    {
        // make sure algae arm is at the top
        public boolean reachedTop = true;
    }

    public default void updateInputs(AlgaeRemoverSensorIOInputs inputs)
    {
    }
}
