package frc.robot.subsystems.usablename;

import org.littletonrobotics.junction.AutoLog;

/**
 * base class of what VisionIO interface should be like,
 * (will most certainly be modified in the future as this is
 * purely an example)
 */
public interface UsableNameIO
{
    /**
     * record containing the data from coprocessor
     */
    @AutoLog
    public class UsableNameIOData
    {
        float yawRadians = Float.NaN;
        boolean detected = false;
        boolean connected = false;
    }
    /**
     * example function of Vision
     * @return data from coprocessor
     */
    public UsableNameIODataAutoLogged getVisionData();
}
