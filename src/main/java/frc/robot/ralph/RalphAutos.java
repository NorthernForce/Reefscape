package frc.robot.ralph;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;

public class RalphAutos
{
    public static void addAutoRoutines(RalphContainer container)
    {
        container.getDashboard().addAutoRoutine("S1.LEAVE", new PathPlannerAuto("S1.LEAVE"));
        container.getDashboard().addAutoRoutine("S1.PLACE", new PathPlannerAuto("S1.PLACE"));
        container.getDashboard().addAutoRoutine("S2.LEAVE", new PathPlannerAuto("S2.LEAVE"));
        container.getDashboard().addAutoRoutine("S2.PLACE", new PathPlannerAuto("S2.PLACE"));
        container.getDashboard().addAutoRoutine("S3.LEAVE", new PathPlannerAuto("S3.LEAVE"));
        container.getDashboard().addAutoRoutine("S3.PLACE", new PathPlannerAuto("S3.PLACE"));
    }
    
    public static void addNamedCommands(RalphContainer container)
    {
        NamedCommands.registerCommand("GoToL4Goal", container.getSuperstructure().getGoToGoalCommand(SuperstructureGoal.L4));
        NamedCommands.registerCommand("GoToL3Goal", container.getSuperstructure().getGoToGoalCommand(SuperstructureGoal.L3));
        NamedCommands.registerCommand("GoToL2Goal", container.getSuperstructure().getGoToGoalCommand(SuperstructureGoal.L2));
        NamedCommands.registerCommand("GoToL1Goal", container.getSuperstructure().getGoToGoalCommand(SuperstructureGoal.L1));
        NamedCommands.registerCommand("GoToIntakeGoal",
        container.getSuperstructure().getGoToGoalCommand(SuperstructureGoal.CORAL_STATION));
        NamedCommands.registerCommand("Intake", container.getRollers().getCoralIntakeCommand(true));
        NamedCommands.registerCommand("Outtake", container.getRollers().getOuttakeCommand());
    }
}
