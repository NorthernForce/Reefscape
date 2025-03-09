package frc.robot.subsystems.dashboard;

import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;

/**
 * Dashboard IO for the FWC dashboard.
 */
public class DashboardIOFWC implements DashboardIO
{
    private final NetworkTable table;
    private final LoggedDashboardChooser<PathPlannerAuto> autoChooser;
    private final DoublePublisher stagePublisher;
    private final DoubleArrayPublisher autoPosePublisher;
    private final DoubleArrayPublisher autoPathPublisher;
    private final DoubleArrayPublisher posePublisher;
    private final DoublePublisher matchTimePublisher;
    private final DoubleSubscriber innerElevatorTargetPosition;
    private final DoubleSubscriber outerElevatorTargetPosition;
    private final DoublePublisher innerElevatorPosition;
    private final DoublePublisher outerElevatorPosition;
    private final BooleanPublisher hasCoralPublisher;
    private final BooleanPublisher hasAlgaePublisher;
    private PathPlannerAuto previousAuto;
    private Alliance previousAlliance;

    /**
     * Creates a new DashboardIOFWC. This connects to the FWC dashboard using "FWC"
     * as the network table name. The dashboard will be hosted on port 5800. The
     * path to the dashboard files is either "./npm-dash/dist" if the code is
     * running in simulation, or "/home/lvuser/npm-dash" if the code is running on
     * the robot.
     */
    public DashboardIOFWC()
    {
        WebServer.start(5800, Utils.isSimulation() ? "./npm-dash/dist" : "/home/lvuser/deploy/npm-dash");
        autoChooser = new LoggedDashboardChooser<PathPlannerAuto>("AutoChooser");
        table = NetworkTableInstance.getDefault().getTable("/FWC");
        stagePublisher = table.getDoubleTopic("selectedTab").publish();
        table.getBooleanTopic("connected").publish().set(true);
        autoPosePublisher = table.getDoubleArrayTopic("AutoPose").publish();
        autoPathPublisher = table.getDoubleArrayTopic("AutoPath").publish();
        posePublisher = table.getDoubleArrayTopic("Pose").publish();
        matchTimePublisher = table.getDoubleTopic("MatchTime").publish();
        innerElevatorTargetPosition = table.getDoubleTopic("InnerElevator/TargetPosition").subscribe(0);
        outerElevatorTargetPosition = table.getDoubleTopic("OuterElevator/TargetPosition").subscribe(0);
        innerElevatorPosition = table.getDoubleTopic("InnerElevator/Position").publish();
        outerElevatorPosition = table.getDoubleTopic("OuterElevator/Position").publish();
        hasCoralPublisher = table.getBooleanTopic("HasCoral").publish();
        hasAlgaePublisher = table.getBooleanTopic("HasAlgae").publish();
    }

    @Override
    public void addRoutine(String name, PathPlannerAuto command, boolean defaultOption)
    {
        if (defaultOption)
        {
            autoChooser.addDefaultOption(name, command);
        } else
        {
            autoChooser.addOption(name, command);
        }
    }

    @Override
    public void setStage(DashboardIOStage stage)
    {
        stagePublisher.set(stage.ordinal());
    }

    @Override
    public void updatePose(Pose2d pose)
    {
        posePublisher.set(new double[]
        { pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getRotation().getRadians() });
    }

    @Override
    public void updateInputs(DashboardIOInputs inputs)
    {
        var pose = FieldConstants.convertPoseByAlliance(autoChooser.get().getStartingPose());
        autoPosePublisher.set(new double[]
        { pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getRotation().getRadians() });
        if (previousAuto != autoChooser.get() || previousAlliance != FieldConstants.getAlliance())
        {
            try
            {
                var paths = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.get().getName());
                ArrayList<Translation2d> pathPoints = new ArrayList<>();
                for (PathPlannerPath path : paths)
                {
                    for (PathPoint pathPoint : path.getAllPathPoints())
                    {
                        pathPoints.add(FieldConstants.convertTranslationByAlliance(pathPoint.position));
                    }
                }
                double[] points = new double[pathPoints.size() * 2];
                for (int i = 0; i < pathPoints.size(); i++)
                {
                    points[i * 2] = pathPoints.get(i).getX();
                    points[i * 2 + 1] = pathPoints.get(i).getY();
                }
                autoPathPublisher.set(points);
            } catch (Exception e)
            {
                e.printStackTrace();
            }
        }
        previousAuto = autoChooser.get();
        inputs.innerElevatorTargetPosition = Inches.of(innerElevatorTargetPosition.get());
        inputs.outerElevatorTargetPosition = Inches.of(outerElevatorTargetPosition.get());
    }

    @Override
    public void setHasCoral(boolean hasCoral)
    {
        hasCoralPublisher.set(hasCoral);
    }

    @Override
    public void setHasAlgae(boolean hasAlgae)
    {
        hasAlgaePublisher.set(hasAlgae);
    }

    @Override
    public PathPlannerAuto getSelectedRoutine()
    {
        return autoChooser.get();
    }

    @Override
    public void addCommand(String name, Command command)
    {
        SmartDashboard.putData(name, command);
    }

    @Override
    public void setTime(double time)
    {
        matchTimePublisher.set(time);
    }

    @Override
    public void setInnerElevatorPosition(Distance position)
    {
        innerElevatorPosition.set(position.in(Inches));
    }

    @Override
    public void setOuterElevatorPosition(Distance position)
    {
        outerElevatorPosition.set(position.in(Inches));
    }
}
