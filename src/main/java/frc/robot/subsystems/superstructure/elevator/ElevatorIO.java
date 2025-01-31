package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;

/**
 * ElevatorIO is an interface that defines the methods that an elevator must
 * implement.
 */

public interface ElevatorIO
{

    /**
     * inputs for the elevator
     */

    @AutoLog
    public static class ElevatorIOInputs
    {
        public Temperature temperature = Fahrenheit.of(0);
        public Distance position = Meters.of(0);
        public boolean present = false;
        public Current current = Amps.of(0);
    }

    /**
     * Sets the target position of the elevator
     * 
     * @param height the height to set the elevator to
     */

    public default void setTargetPosition(Distance height)
    {
    }

    public default void resetPosition()
    {
    }

    /**
     * Stops the elevator
     */

    public default void stop()
    {
    }

    /**
     * Updates the inputs for the elevator
     * 
     * @param inputs the inputs to update
     */

    public default void updateInputs(ElevatorIOInputs inputs)
    {
    }
}
