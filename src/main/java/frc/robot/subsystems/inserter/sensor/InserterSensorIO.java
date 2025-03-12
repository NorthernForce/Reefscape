package frc.robot.subsystems.inserter.sensor;

import org.littletonrobotics.junction.AutoLog;

/**
 * The IO for the inserter algae and coral sensor.
 */

public interface InserterSensorIO
{

    /**
     * The inputs for the inserter algae and coral sensor.
     */

    @AutoLog
    public static class InserterSensorIOInputs
    {
        public boolean hasPiece = false;
    }

    /**
     * Updates the inputs for the inserter algae and coral sensor.
     * 
     * @param inputs The inputs to update.
     */

    public default void updateInputs(InserterSensorIOInputs inputs)
    {
    }
}
