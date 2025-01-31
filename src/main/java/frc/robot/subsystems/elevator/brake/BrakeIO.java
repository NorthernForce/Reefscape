package frc.robot.subsystems.elevator.brake;

import org.littletonrobotics.junction.AutoLog;

/**
 * BrakeIO is an interface that defines the methods that a brake must implement.
 */

public interface BrakeIO
{

    /**
     * inputs for the brake
     */

    @AutoLog
    public static class BrakeIOInputs
    {
        public boolean isOn = false;
    }

    /**
     * Sets the brake to on or off
     * 
     * @param on true to set the brake on, false to set the brake off
     */

    public void setBreak(boolean on);

    /**
     * Updates the inputs for the brake
     * 
     * @param inputs the inputs to update
     */

    public default void updateInputs(BrakeIOInputs inputs)
    {
    }
}
