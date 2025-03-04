package frc.robot.ralph;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;

public class RalphAutos
{
    public static void addAutoRoutines(RalphContainer container)
    {
        container.getDashboard().addAutoRoutine("BLUE.LEAVE", new PathPlannerAuto("BLUE.LEAVE"));
        container.getDashboard().addAutoRoutine("BLUE.PLACE.I", new PathPlannerAuto("BLUE.PLACE.I"));
        container.getDashboard().addDefaultAutoRoutine("CENTER.LEAVE", new PathPlannerAuto("CENTER.LEAVE"));
        container.getDashboard().addAutoRoutine("CENTER.PLACE.G", new PathPlannerAuto("CENTER.PLACE.G"));
        container.getDashboard().addAutoRoutine("RED.LEAVE", new PathPlannerAuto("RED.LEAVE"));
        container.getDashboard().addAutoRoutine("RED.PLACE.E", new PathPlannerAuto("RED.PLACE.E"));
    }

    public static void addNamedCommands(RalphContainer container)
    {
        NamedCommands.registerCommand("GoToL4Goal",
                container.getSuperstructure().getGoToGoalCommand(SuperstructureGoal.L4));
        NamedCommands.registerCommand("GoToL3Goal",
                container.getSuperstructure().getGoToGoalCommand(SuperstructureGoal.L3));
        NamedCommands.registerCommand("GoToL2Goal",
                container.getSuperstructure().getGoToGoalCommand(SuperstructureGoal.L2));
        NamedCommands.registerCommand("GoToL1Goal",
                container.getSuperstructure().getGoToGoalCommand(SuperstructureGoal.L1));
        NamedCommands.registerCommand("GoToIntakeGoal",
                container.getSuperstructure().getGoToGoalCommand(SuperstructureGoal.CORAL_STATION));
        NamedCommands.registerCommand("Intake", container.getRollers().getCoralIntakeCommand(true));
        NamedCommands.registerCommand("Outtake", container.getRollers().getOuttakeCommand());
    }
}
