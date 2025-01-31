package frc.robot.zippy.dashboard;

import edu.wpi.first.net.WebServer;

public class ZippyDashboardIOFWC implements ZippyDashboardIO
{
    public ZippyDashboardIOFWC()
    {
        WebServer.start(5800, "/home/lvuser/npm-dash");
    }
    @Override
    public void updateInputs(ZippyDashboardIOInputs inputs)
    {
    }
}
