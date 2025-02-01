package frc.robot.zippy.dashboard;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;

public interface ZippyDashboardIO
{
    public static enum ZippyDashboardIOStage
    {
        TELEOP, AUTO, SETTINGS
    }

    @AutoLog
    public static class ZippyDashboardIOInputs
    {
    }

    public default void addCommand(String name, Command command, boolean defaultOption)
    {
    }

    public default void setStage(ZippyDashboardIOStage stage)
    {
    }

    public default void updateInputs(ZippyDashboardIOInputs inputs)
    {
    }
}
