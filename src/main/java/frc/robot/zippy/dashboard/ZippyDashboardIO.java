package frc.robot.zippy.dashboard;

import org.littletonrobotics.junction.AutoLog;

public interface ZippyDashboardIO
{
    @AutoLog
    public static class ZippyDashboardIOInputs
    {
    }

    public default void updateInputs(ZippyDashboardIOInputs inputs)
    {
    }
}
