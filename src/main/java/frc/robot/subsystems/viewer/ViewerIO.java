package frc.robot.subsystems.viewer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public interface ViewerIO
{
    @AutoLog
    public static class ViewerIOInputs
    {
        public boolean connected = false;
        public boolean hasPostInImage = false;
        public Distance postOffset = Meters.of(0.0);
        public Distance postDistance = Meters.of(0);
        public Distance centerDistance = Meters.of(0);
    }

    /**
     * Updates the inputs.
     * 
     * @param inputs
     */
    public default void updateInputs(ViewerIOInputs inputs)
    {
    }
}
