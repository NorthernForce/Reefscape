package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.NFRAutoRoutine;

public interface DashboardIO
{
    public static enum DashboardIOStage
    {
        TELEOP, AUTO, SETTINGS
    }

    @AutoLog
    public static class DashboardIOInputs
    {
    }

    public default void addRoutine(String name, NFRAutoRoutine command, boolean defaultOption)
    {
    }

    public default void setStage(DashboardIOStage stage)
    {
    }

    public default void updateInputs(DashboardIOInputs inputs)
    {
    }

    public default NFRAutoRoutine getSelectedRoutine()
    {
        return null;
    }
}
