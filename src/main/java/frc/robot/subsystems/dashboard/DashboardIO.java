package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.AutoRoutine;

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

    public default void addRoutine(String name, AutoRoutine command, boolean defaultOption)
    {
    }

    public default void setStage(DashboardIOStage stage)
    {
    }

    public default void updateInputs(DashboardIOInputs inputs)
    {
    }

    public default AutoRoutine getSelectedRoutine()
    {
        return null;
    }

    public default void addCommand(String name, Command command)
    {
    }
}
