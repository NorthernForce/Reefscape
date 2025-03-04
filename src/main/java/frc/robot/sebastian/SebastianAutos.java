package frc.robot.sebastian;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class SebastianAutos
{
    public static void addAutoRoutines(SebastianContainer container)
    {
        container.getDashboard().addAutoRoutine("S1.LEAVE", new PathPlannerAuto("S1.LEAVE"));
        container.getDashboard().addAutoRoutine("S1.PLACE", new PathPlannerAuto("S1.PLACE"));
        container.getDashboard().addAutoRoutine("S2.LEAVE", new PathPlannerAuto("S2.LEAVE"));
        container.getDashboard().addAutoRoutine("S2.PLACE", new PathPlannerAuto("S2.PLACE"));
        container.getDashboard().addAutoRoutine("S3.LEAVE", new PathPlannerAuto("S3.LEAVE"));
        container.getDashboard().addAutoRoutine("S3.PLACE", new PathPlannerAuto("S3.PLACE"));
    }
}
