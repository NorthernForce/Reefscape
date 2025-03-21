package frc.robot.subsystems.viewer;

import org.littletonrobotics.junction.AutoLog;

public interface ViewerIO
{
    @AutoLog
    public static class ViewerIOInputs
    {
        public boolean connected = false;
        public double[] postOffsetMeters = new double[0];
        public double[] postDistanceMeters = new double[0];
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
