package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public interface RollersIO
{

    /**
     * The inputs for the rollers.
     */

    @AutoLog
    public static class IntakeIOInputs
    {
        public Temperature motorLeftTemperature;
        public boolean motorLeftPresent;
        public Current motorLeftCurrent;
        public Temperature motorRightTemperature;
        public boolean motorRightPresent;
        public Current motorRightCurrent;
    }

    /**
     * Sets the speed of the rollers.
     * 
     * @param speed The speed to set the rollers to.
     */

    public default void set(double speed)
    {
    }

    /**
     * Updates the inputs for the rollers.
     * 
     * @param inputs The inputs to update.
     */

    public default void updateInputs(IntakeIOInputs inputs)
    {
    }
}
