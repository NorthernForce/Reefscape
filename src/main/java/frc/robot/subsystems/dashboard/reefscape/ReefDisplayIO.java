package frc.robot.subsystems.dashboard.reefscape;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.FieldConstants.ReefLocations;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;

/**
 * Reef display IO for the robot.
 */
public interface ReefDisplayIO
{
    @AutoLog
    public static class ReefDisplayIOInputs
    {
        public ReefLocations reefLocations = ReefLocations.A;
        public ReefLocations stationlocations = ReefLocations.LEFT_CORAL_STATION;
        public int level = 0;
        public boolean isConnected = false;
        public SuperstructureGoal reefGoal = SuperstructureGoal.L1;
        public SuperstructureGoal stationGoal = SuperstructureGoal.CORAL_STATION;
    }

    /**
     * Set the grayed out state of a reef location. This is used to indicate that a
     * coral is already placed in that location.
     * 
     * @param reefLocation the reef location to set the grayed out state of
     * @param level        the level to set the grayed out state of (0-3)
     * @param grayedOut    the grayed out state to set
     */
    public default void setGrayedOut(ReefLocations reefLocation, int level, boolean grayedOut)
    {
    }

    /**
     * Updates the inputs of the reef display IO
     * 
     * @param inputs the inputs class containing reef display IO inputs
     */
    public default void updateInputs(ReefDisplayIOInputs inputs)
    {
    }
}
