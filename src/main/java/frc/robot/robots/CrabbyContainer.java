package frc.robot.robots;

import java.util.Map;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
public class CrabbyContainer implements NFRRobotContainer{

    public Map<String, Command> getAutonomousOptions() {
        return Map.of();
    }

    public Map<String, Pose2d> getStartingLocations() {
        return Map.of();
    }
    public Pair<String, Command> getDefaultAutonomous() {
        return Pair.of("nothing", Commands.none());
    }
    public void setInitialPose(Pose2d pose) {
        
    }

    public void bindOI(GenericHID driverHID, GenericHID manipulatorHID) {

    }

}
