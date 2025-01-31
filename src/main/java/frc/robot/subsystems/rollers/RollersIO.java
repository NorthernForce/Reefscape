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
        public Temperature motorOneTemperature;
        public boolean motorOnePresent;
        public Current motorOneCurrent;
        public Temperature motorTwoTemperature;
        public boolean motorTwoPresent;
        public Current motorTwoCurrent;
    }

    /**
     * Sets the speed of the rollers.
     * 
     * @param speed The speed to set the rollers to.
     */

    public void set(double speed);

    /**
     * Updates the inputs for the rollers.
     * 
     * @param inputs The inputs to update.
     */

    public void updateInputs(IntakeIOInputs inputs);
}
