package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.NFRAutoRoutine;

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
        public Distance innerElevatorTargetPosition;
        public Distance outerElevatorTargetPosition;
    }

    /**
     * Adds a routine to the dashboard
     * 
     * @param name          the name of the routine
     * @param command       the command to run
     * @param defaultOption whether or not this is the default option
     */
    public default void addRoutine(String name, NFRAutoRoutine command, boolean defaultOption)
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

    public default void updatePose(Pose2d pose)
    {
    }

    /**
     * Gets the selected routine
     * 
     * @return the selected routine
     */
    public default NFRAutoRoutine getSelectedRoutine()
    {
        return null;
    }

    public default void addCommand(String name, Command command)
    {
    }

    public default void setTime(double time)
    {
    }

    public default void setInnerElevatorPosition(Distance position)
    {
    }

    public default void setOuterElevatorPosition(Distance position)
    {
    }

    public default void setHasCoral(boolean hasCoral)
    {
    }

    public default void setHasAlgae(boolean hasAlgae)
    {
    }
}
