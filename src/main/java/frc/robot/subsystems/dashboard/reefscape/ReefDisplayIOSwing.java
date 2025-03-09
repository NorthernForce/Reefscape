package frc.robot.subsystems.dashboard.reefscape;

import java.util.Arrays;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.FieldConstants.ReefLocations;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;

/**
 * ReefDisplayIO for the Swing dashboard.
 */
public class ReefDisplayIOSwing implements ReefDisplayIO
{
    private final String name;
    private final NetworkTable table;
    private final IntegerSubscriber selectedPoint, stationSelected;
    private final BooleanArrayPublisher grayedOutPublisher;
    private final boolean[] grayedOut = new boolean[48];

    /**
     * Creates a new ReefDisplayIOSwing
     * 
     * @param name the name of the network table to use
     */
    public ReefDisplayIOSwing(String name)
    {
        this.name = name;
        this.table = NetworkTableInstance.getDefault().getTable(name);
        this.selectedPoint = table.getIntegerTopic("reefChoice").subscribe(0);
        stationSelected = table.getIntegerTopic("stationChoice").subscribe(0);
        Arrays.fill(this.grayedOut, false);
        this.grayedOutPublisher = table.getBooleanArrayTopic("grayedOut").publish();
    }

    @Override
    public void updateInputs(ReefDisplayIOInputs inputs)
    {
        int selected = (int) selectedPoint.get();
        SuperstructureGoal goal = ((selected) % 9 == 0 || (selected) % 9 == 4) ? SuperstructureGoal.L1
                : (((selected) % 9 == 1 || (selected) % 9 == 5) ? SuperstructureGoal.L2
                        : (((selected) % 9 == 2 || (selected) % 9 == 6) ? SuperstructureGoal.L3
                                : SuperstructureGoal.L4));

        inputs.reefGoal = goal;
        grayedOutPublisher.accept(grayedOut);
        if (selected >= 0 && selected < 4)
        {
            inputs.reefLocations = ReefLocations.E;
        } else if (selected >= 4 && selected < 8)
        {
            inputs.reefLocations = ReefLocations.F;
        } else if (selected == 8)
        {
            inputs.reefLocations = ReefLocations.EF_ALGAE;
        } else if (selected >= 9 && selected < 13)
        {
            inputs.reefLocations = ReefLocations.G;
        } else if (selected >= 13 && selected < 17)
        {
            inputs.reefLocations = ReefLocations.H;
        } else if (selected == 17)
        {
            inputs.reefLocations = ReefLocations.GH_ALGAE;
        } else if (selected >= 18 && selected < 22)
        {
            inputs.reefLocations = ReefLocations.I;
        } else if (selected >= 22 && selected < 26)
        {
            inputs.reefLocations = ReefLocations.J;
        } else if (selected == 26)
        {
            inputs.reefLocations = ReefLocations.IJ_ALGAE;
        } else if (selected >= 27 && selected < 31)
        {
            inputs.reefLocations = ReefLocations.K;
        } else if (selected >= 31 && selected < 35)
        {
            inputs.reefLocations = ReefLocations.L;
        } else if (selected == 35)
        {
            inputs.reefLocations = ReefLocations.KL_ALGAE;
        } else if (selected >= 36 && selected < 40)
        {
            inputs.reefLocations = ReefLocations.A;
        } else if (selected >= 40 && selected < 44)
        {
            inputs.reefLocations = ReefLocations.B;
        } else if (selected == 44)
        {
            inputs.reefLocations = ReefLocations.AB_ALGAE;
        } else if (selected >= 45 && selected < 49)
        {
            inputs.reefLocations = ReefLocations.C;
        } else if (selected >= 49 && selected < 53)
        {
            inputs.reefLocations = ReefLocations.D;
        } else if (selected == 53)
        {
            inputs.reefLocations = ReefLocations.CD_ALGAE;
        }
        int otherSelected = (int) stationSelected.get();
        if (otherSelected == 0)
        {
            inputs.stationlocations = ReefLocations.RIGHT_CORAL_STATION;
            inputs.stationGoal = SuperstructureGoal.CORAL_STATION;
        } else if (selected == 1)
        {
            inputs.stationlocations = ReefLocations.LEFT_CORAL_STATION;
            inputs.stationGoal = SuperstructureGoal.CORAL_STATION;
        }
        inputs.isConnected = false;
        for (var connection : NetworkTableInstance.getDefault().getConnections())
        {
            if (connection.remote_id.startsWith(name))
            {
                inputs.isConnected = true;
                break;
            }
        }
    }

    @Override
    public void setGrayedOut(ReefLocations reefLocation, int level, boolean grayedOut)
    {
        switch (reefLocation)
        {
        case A:
            this.grayedOut[level] = grayedOut;
            break;
        case AB_ALGAE:
            break;
        case B:
            this.grayedOut[level + 4] = grayedOut;
            break;
        case C:
            this.grayedOut[level + 8] = grayedOut;
            break;
        case CD_ALGAE:
            break;
        case D:
            this.grayedOut[level + 12] = grayedOut;
            break;
        case E:
            this.grayedOut[level + 16] = grayedOut;
            break;
        case EF_ALGAE:
            break;
        case F:
            this.grayedOut[level + 20] = grayedOut;
            break;
        case G:
            this.grayedOut[level + 24] = grayedOut;
            break;
        case GH_ALGAE:
            break;
        case H:
            this.grayedOut[level + 28] = grayedOut;
            break;
        case I:
            this.grayedOut[level + 32] = grayedOut;
            break;
        case IJ_ALGAE:
            break;
        case J:
            this.grayedOut[level + 36] = grayedOut;
            break;
        case K:
            this.grayedOut[level + 40] = grayedOut;
            break;
        case KL_ALGAE:
            break;
        case L:
            this.grayedOut[level + 44] = grayedOut;
            break;
        case LEFT_CORAL_STATION:
            break;
        case PROCESSOR_STATION:
            break;
        case RIGHT_CORAL_STATION:
            break;
        default:
            break;
        }
    }
}
