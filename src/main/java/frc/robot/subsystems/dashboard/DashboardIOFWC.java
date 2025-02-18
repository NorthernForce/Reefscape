package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.util.AutoRoutine;

public class DashboardIOFWC implements DashboardIO
{
    private final NetworkTable table;
    private final LoggedDashboardChooser<AutoRoutine> autoChooser;
    private final DoublePublisher stagePublisher;
    private final DoubleArrayPublisher autoPosePublisher;
    private final DoubleArrayPublisher autoPathPublisher;
    private final DoubleArrayPublisher posePublisher;

    public DashboardIOFWC()
    {
        WebServer.start(5800, Utils.isSimulation() ? "./npm-dash/dist" : "/home/lvuser/npm-dash");
        autoChooser = new LoggedDashboardChooser<AutoRoutine>("AutoChooser");
        table = NetworkTableInstance.getDefault().getTable("/FWC");
        stagePublisher = table.getDoubleTopic("selectedTab").publish();
        table.getBooleanTopic("connected").publish().set(true);
        autoPosePublisher = table.getDoubleArrayTopic("AutoPose").publish();
        autoPathPublisher = table.getDoubleArrayTopic("AutoPath").publish();
        posePublisher = table.getDoubleArrayTopic("Pose").publish();
    }

    @Override
    public void addRoutine(String name, AutoRoutine command, boolean defaultOption)
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
        var pose = autoChooser.get().startPose();
        pose = FieldConstants.convertPoseByAlliance(pose, FieldConstants.getAlliance());
        autoPosePublisher.set(new double[]
        { pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getRotation().getRadians() });
        var path = autoChooser.get().waypoints().clone();
        for (int i = 0; i < path.length; i++)
        {
            path[i] = FieldConstants.convertTranslationByAlliance(path[i], FieldConstants.getAlliance());
        }
        var pathArray = new double[path.length * 2];
        for (int i = 0; i < path.length; i++)
        {
            pathArray[i * 2] = path[i].getX();
            pathArray[i * 2 + 1] = path[i].getY();
        }
        autoPathPublisher.set(pathArray);
    }

    @Override
    public AutoRoutine getSelectedRoutine()
    {
        return autoChooser.get();
    }

    @Override
    public void addCommand(String name, Command command)
    {
        SmartDashboard.putData("/FWC/" + name, command);
    }
}
