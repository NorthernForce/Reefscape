package frc.robot.subsystems.viewer;

import org.littletonrobotics.junction.AutoLog;

public interface ViewerIO
{
    @AutoLog
    public static class ViewerIOInputs
    {
        public boolean connected = false;
        public boolean postDetected = false;
        public double postXOffset = Float.NaN;
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
