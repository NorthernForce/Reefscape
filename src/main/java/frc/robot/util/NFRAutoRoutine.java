package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public record NFRAutoRoutine(Command command, Translation2d[] waypoints, Supplier<Pose2d> startPose) {
    public NFRAutoRoutine
    {
        if (waypoints.length < 2)
        {
            throw new IllegalArgumentException("AutoRoutine must have at least 2 waypoints");
        }
    }
}
