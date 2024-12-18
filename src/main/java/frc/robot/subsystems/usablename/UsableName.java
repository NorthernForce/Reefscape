package frc.robot.subsystems.usablename;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Xavier;

public class UsableName extends SubsystemBase implements UsableNameIO
{
    private final Xavier xavier;
    public UsableName(Xavier xavier)
    {
        this.xavier = xavier;
    }
    public UsableNameIODataAutoLogged getVisionData()
    {
        float yaw = xavier.getYawRadians();
        var data = new UsableNameIODataAutoLogged();
        data.connected = xavier.isConnected();
        data.detected = yaw == Float.NaN;
        data.yawRadians = yaw;
        return data;
    }
    public boolean isConnected()
    {
        return xavier.isConnected();
    }
}
