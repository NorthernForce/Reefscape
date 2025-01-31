package frc.robot.subsystems.superstructure.elevator.sensor;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the elevator sensor IO
 */
public interface ElevatorSensorIO
{
    @AutoLog
    public static class ElevatorSensorIOInputs
    {
        public boolean isAtBottom = false;
    }

    /**
     * Updates the inputs for the elevator
     * 
     * @param inputs the inputs to update
     */
    public default void updateInputs(ElevatorSensorIOInputs inputs)
    {
    }
}
