package frc.robot.subsystems.inserter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface InserterIO
{

    /**
     * The inputs for the rollers.
     */

    @AutoLog
    public static class InserterIOInputs
    {
        public Temperature motorTemperature;
        public boolean motorPresent;
        public Current motorCurrent;
        public Voltage motorVoltage;
        public AngularVelocity motorVelocity;
        public Angle motorPosition;
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

    public default void updateInputs(InserterIOInputs inputs)
    {
    }
}
