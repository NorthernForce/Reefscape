package frc.robot.ralph;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.util.NFRAutoRoutine;

public class RalphAutos
{
    public static void addAutoRoutines(RalphContainer container)
    {
        container.getDashboard().addAutoRoutine("Do Nothing", doNothing(container));
        container.getDashboard().addDefaultAutoRoutine("Simple Backup", simpleBackup(container));
    }

    public static NFRAutoRoutine doNothing(RalphContainer container)
    {
        return new NFRAutoRoutine(Commands.none(), new Translation2d[]
        { Translation2d.kZero, Translation2d.kZero }, () -> Pose2d.kZero);
    }

    public static NFRAutoRoutine simpleBackup(RalphContainer container)
    {
        Pose2d startingPose = new Pose2d(7.5, 4.06, Rotation2d.kZero);
        return new NFRAutoRoutine(container.getDrive().backup(Seconds.of(4), -0.3), new Translation2d[]
        { new Translation2d(7.5, 4.06), new Translation2d(7.5, 4.06) },
                () -> FieldConstants.convertPoseByAlliance(startingPose));
    }
}
