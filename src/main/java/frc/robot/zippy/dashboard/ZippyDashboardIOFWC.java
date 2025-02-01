package frc.robot.zippy.dashboard;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class ZippyDashboardIOFWC implements ZippyDashboardIO
{
    private final NetworkTable table;
    private final LoggedDashboardChooser<Command> autoChooser;
    private final DoublePublisher stagePublisher;

    public ZippyDashboardIOFWC()
    {
        WebServer.start(5800, Utils.isSimulation() ? "./npm-dash/dist" : "/home/lvuser/npm-dash");
        autoChooser = new LoggedDashboardChooser<Command>("/AutoChooser");
        table = NetworkTableInstance.getDefault().getTable("/FWC");
        stagePublisher = table.getDoubleTopic("selectedTab").publish();
        table.getBooleanTopic("connected").publish().set(true);
    }

    @Override
    public void addCommand(String name, Command command, boolean defaultOption)
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
    public void setStage(ZippyDashboardIOStage stage)
    {
        stagePublisher.set(stage.ordinal());
    }

    @Override
    public void updateInputs(ZippyDashboardIOInputs inputs)
    {
    }
}
