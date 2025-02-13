package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.AutoRoutine;

/**
 * Dashboard IO for the robot.
 */
public interface DashboardIO
{
    /**
     * The stage of the dashboard
     */
    public static enum DashboardIOStage
    {
        TELEOP, AUTO, SETTINGS
    }

    @AutoLog
    public static class DashboardIOInputs
    {
    }

    /**
     * Adds a routine to the dashboard
     * 
     * @param name          the name of the routine
     * @param command       the command to run
     * @param defaultOption whether or not this is the default option
     */
    public default void addRoutine(String name, AutoRoutine command, boolean defaultOption)
    {
    }

    /**
     * Sets the stage of the dashboard
     * 
     * @param stage the stage to set
     */
    public default void setStage(DashboardIOStage stage)
    {
    }

    /**
     * Updates the inputs of the dashboard
     * 
     * @param inputs the inputs class containing dashboard IO inputs
     */
    public default void updateInputs(DashboardIOInputs inputs)
    {
    }

    /**
     * Gets the selected routine
     * 
     * @return the selected routine
     */
    public default AutoRoutine getSelectedRoutine()
    {
        return null;
    }
}
