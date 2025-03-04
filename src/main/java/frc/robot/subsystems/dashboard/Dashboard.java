package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefLocations;
import frc.robot.FieldConstants.ReefPositions.ReefSide;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;
import frc.robot.subsystems.dashboard.reefscape.ReefDisplayIO;
import frc.robot.subsystems.dashboard.reefscape.ReefDisplayIOInputsAutoLogged;
import frc.robot.util.NFRAutoRoutine;

import static edu.wpi.first.units.Units.Meters;

/**
 * Subsystem for the dashboard.
 */
public class Dashboard extends SubsystemBase
{
    private final DashboardIO m_io;
    private final DashboardIOInputsAutoLogged m_input;
    private final ReefDisplayIO reefDisplayIO;
    private final ReefDisplayIOInputsAutoLogged reefDisplayInputs;
    private boolean useBeamBreak = true;

    /**
     * Constructs a new Dashboard.
     */
    public Dashboard(ReefDisplayIO displayIO, DashboardIO io)
    {
        this.m_io = io;
        this.m_input = new DashboardIOInputsAutoLogged();
        this.reefDisplayIO = displayIO;
        this.reefDisplayInputs = new ReefDisplayIOInputsAutoLogged();
    }

    /**
     * Gets the target pose for the reef location.
     * 
     * @return The target pose.
     */
    @AutoLogOutput
    public Pose2d getTargetPoseLeft(Pose2d currentPose)
    {
        Pose2d closestTranslation = FieldConstants.ReefPositions.SIDES[0].left();
        for (ReefSide pose : FieldConstants.ReefPositions.SIDES)
        {
            if (pose.getDistanceFromLeft(currentPose).in(Meters) < pose.getDistanceFromLeft(closestTranslation)
                    .in(Meters))
            {
                closestTranslation = pose.left();
            }
        }
        return FieldConstants.convertPoseByAlliance(closestTranslation, FieldConstants.getAlliance());
    }

    public Pose2d getTargetPoseCenter(Pose2d currentPose)
    {
        Pose2d closestTranslation = FieldConstants.ReefPositions.SIDES[0].center();
        for (ReefSide pose : FieldConstants.ReefPositions.SIDES)
        {
            if (pose.getDistanceFromCenter(currentPose).in(Meters) < pose.getDistanceFromCenter(closestTranslation)
                    .in(Meters))
            {
                closestTranslation = pose.center();
            }
        }
        return FieldConstants.convertPoseByAlliance(closestTranslation, FieldConstants.getAlliance());
    }

    public Pose2d getTargetPoseRight(Pose2d currentPose)
    {
        Pose2d closestTranslation = FieldConstants.ReefPositions.SIDES[0].right();
        for (ReefSide pose : FieldConstants.ReefPositions.SIDES)
        {
            if (pose.getDistanceFromRight(currentPose).in(Meters) < pose.getDistanceFromRight(closestTranslation)
                    .in(Meters) || closestTranslation == null)
            {
                closestTranslation = pose.right();
            }
        }
        return FieldConstants.convertPoseByAlliance(closestTranslation, FieldConstants.getAlliance());
    }

    /**
     * Adds an auto routine to the dashboard.
     * 
     * @param name    Auto routine name (Descriptive for drivers please)
     * @param command Auto routine command
     */
    public void addAutoRoutine(String name, NFRAutoRoutine command)
    {
        m_io.addRoutine(name, command, false);
    }

    /**
     * Adds a default auto routine to the dashboard.
     * 
     * @param name    Auto routine name (Descriptive for drivers please)
     * @param command Auto routine command
     */
    public void addDefaultAutoRoutine(String name, NFRAutoRoutine command)
    {
        m_io.addRoutine(name, command, true);
    }

    /**
     * sets the beam break sensor to be used or not
     * 
     * @param useBeamBreak
     */

    public void toggleBeamBreak()
    {
        useBeamBreak = !useBeamBreak;
        SmartDashboard.putBoolean("Use Beam Break", useBeamBreak);
    }

    /**
     * gets the beam break sensor to be used or not
     * 
     * @return
     */

    public boolean getUseBeamBreak()
    {
        return useBeamBreak;
    }

    /**
     * Sets the stage of the dashboard to auto. This only changes the display stage
     * when toggle is on.
     */
    public void setAutoStage()
    {
        m_io.setStage(DashboardIO.DashboardIOStage.AUTO);
    }

    /**
     * Sets the stage of the dashboard to teleop. This only changes the display
     * stage when toggle is on.
     */
    public void setTeleopStage()
    {
        m_io.setStage(DashboardIO.DashboardIOStage.TELEOP);
    }

    /**
     * Sets the stage of the dashboard to settings. This only changes the display
     * stage when toggle is on.
     */
    public void setSettingsStage()
    {
        m_io.setStage(DashboardIO.DashboardIOStage.SETTINGS);
    }

    public void setResetEncodersCommand(Command command)
    {
        m_io.addCommand("ResetSwerveWheels", command);
    }

    public void setInnerElevatorGoToPosition(Command command)
    {
        m_io.addCommand("InnerElevator/GoToPosition", command);
    }

    public void setOuterElevatorGoToPosition(Command command)
    {
        m_io.addCommand("OuterElevator/GoToPosition", command);
    }

    public void setInnerElevatorPosition(Distance position)
    {
        m_io.setInnerElevatorPosition(position);
    }

    public void setOuterElevatorPosition(Distance position)
    {
        m_io.setOuterElevatorPosition(position);
    }

    public Distance getInnerElevatorTargetPosition()
    {
        return m_input.innerElevatorTargetPosition;
    }

    public Distance getOuterElevatorTargetPosition()
    {
        return m_input.outerElevatorTargetPosition;
    }

    public SuperstructureGoal getGoal()
    {
        switch (reefDisplayInputs.level)
        {
        case 0:
            return SuperstructureGoal.L1;
        case 1:
            return SuperstructureGoal.L2;
        case 2:
            return SuperstructureGoal.L3;
        case 3:
            return SuperstructureGoal.L4;
        default:
            return SuperstructureGoal.L1;
        }
    }

    public void updatePose(Pose2d pose)
    {
        m_io.updatePose(pose);
    }

    @Override
    public void periodic()
    {
        m_io.updateInputs(m_input);
        Logger.processInputs(getName() + "/Dashboard", m_input);
        reefDisplayIO.updateInputs(reefDisplayInputs);
        Logger.processInputs(getName() + "/ReefDisplayIO", reefDisplayInputs);
        m_io.setTime(DriverStation.getMatchTime());
    }

    /**
     * Gets the selected auto routine. It will return the default auto routine if no
     * routine is selected.
     * 
     * @return The selected auto routine.
     */
    public NFRAutoRoutine getRoutine()
    {
        return m_io.getSelectedRoutine();
    }

    public SuperstructureGoal getSuperstructureGoalForReef()
    {
        return reefDisplayInputs.reefGoal;
    }

    public SuperstructureGoal getSuperstructureGoalForStation()
    {
        return reefDisplayInputs.stationGoal;
    }

    public ReefLocations getStationLocation()
    {
        return reefDisplayInputs.stationlocations;
    }

    public void setHasCoral(boolean hasCoral)
    {
        m_io.setHasCoral(hasCoral);
    }

    public void setHasAlgae(boolean hasVision)
    {
        m_io.setHasAlgae(hasVision);
    }

    @AutoLogOutput
    public Pose2d getStationTargetPose()
    {
        return FieldConstants.convertPoseByAlliance(
                FieldConstants.REEF_POSITIONS.get(reefDisplayInputs.stationlocations), FieldConstants.getAlliance());
    }
}