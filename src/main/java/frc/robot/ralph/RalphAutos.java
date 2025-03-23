package frc.robot.ralph;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Commands;

public class RalphAutos
{
    public static void addAutoRoutines(RalphContainer container)
    {
        container.getDashboard().addAutoRoutine("Simple Leave", container.getDriveSimple());
        container.getDashboard().addDefaultAutoRoutine("Simple Leave and PLACE", container.getDrivePlace());
        container.getDashboard().addAutoRoutine("LEFT.LEAVE", new PathPlannerAuto("LEFT.LEAVE"));
        container.getDashboard().addAutoRoutine("LEFT.PLACE.I", new PathPlannerAuto("LEFT.PLACE.I"));
        container.getDashboard().addAutoRoutine("CENTER.LEAVE", new PathPlannerAuto("CENTER.LEAVE"));
        container.getDashboard().addAutoRoutine("CENTER.PLACE.G", new PathPlannerAuto("CENTER.PLACE.G"));
        container.getDashboard().addAutoRoutine("RIGHT.LEAVE", new PathPlannerAuto("RIGHT.LEAVE"));
        container.getDashboard().addAutoRoutine("RIGHT.PLACE.E", new PathPlannerAuto("RIGHT.PLACE.E"));
        container.getDashboard().addAutoRoutine("LEFT.J.K.L", new PathPlannerAuto("LEFT.J.K.L"));
        container.getDashboard().addAutoRoutine("RIGHT.E.D.C", new PathPlannerAuto("RIGHT.E.D.C"));
    }

    public static void addNamedCommands(RalphContainer container)
    {
        NamedCommands.registerCommand("GoToL4Goal", container.goToL4());
        NamedCommands.registerCommand("GoToL3Goal", container.goToL3());
        NamedCommands.registerCommand("GoToL2Goal", container.goToL2());
        NamedCommands.registerCommand("GoToL1Goal", container.goToL1());
        NamedCommands.registerCommand("GoToIntake", container.goToIntakeForAuto());
        NamedCommands.registerCommand("Intake", container.intakeCoral());
        NamedCommands.registerCommand("Outtake", container.outtakeCoral().withTimeout(0.5));
        NamedCommands.registerCommand("DriveToCloseLeft", container.driveToLeftReef());
        NamedCommands.registerCommand("DriveToCloseRight", container.driveToRightReef());
    }
}
