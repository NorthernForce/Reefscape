package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;

public class FieldConstants
{
    public static final Distance FIELD_LENGTH = Meters.of(17.55);
    public static final Distance FIELD_WIDTH = Meters.of(8.05);

    /**
     * Enum for the reef locations
     */
    public static enum ReefLocations
    {
        A, B, C, D, E, F, G, H, I, J, K, L, AB_ALGAE, CD_ALGAE, EF_ALGAE, GH_ALGAE, IJ_ALGAE, KL_ALGAE,
        LEFT_CORAL_STATION, RIGHT_CORAL_STATION, PROCESSOR_STATION
    }

    public static class ReefRotations
    {
        public static final Rotation2d AB_ROTATION = Rotation2d.fromDegrees(0);
        public static final Rotation2d CD_ROTATION = Rotation2d.fromDegrees(60);
        public static final Rotation2d EF_ROTATION = Rotation2d.fromDegrees(120);
        public static final Rotation2d GH_ROTATION = Rotation2d.fromDegrees(180);
        public static final Rotation2d IJ_ROTATION = Rotation2d.fromDegrees(240);
        public static final Rotation2d KL_ROTATION = Rotation2d.fromDegrees(300);
    }

    /**
     * All poses are BLUE relative
     */
    public static class ReefPositions
    {
        public record ReefSide(Pose2d left, Pose2d center, Pose2d right) {
            public Distance getDistanceFromCenter(Pose2d pose)
            {
                return Meters.of(center.getTranslation().getDistance(pose.getTranslation()));
            }

            public Distance getDistanceFromCenter(Translation2d pose)
            {
                return Meters.of(center.getTranslation().getDistance(pose));
            }

            public Distance getDistanceFromLeft(Pose2d pose)
            {
                return Meters.of(left.getTranslation().getDistance(pose.getTranslation()));
            }

            public Distance getDistanceFromLeft(Translation2d pose)
            {
                return Meters.of(left.getTranslation().getDistance(pose));
            }

            public Distance getDistanceFromRight(Pose2d pose)
            {
                return Meters.of(right.getTranslation().getDistance(pose.getTranslation()));
            }

            public Distance getDistanceFromRight(Translation2d pose)
            {
                return Meters.of(right.getTranslation().getDistance(pose));
            }
        }

        public static final Pose2d A = new Pose2d(3.15, 4.18, ReefRotations.AB_ROTATION);
        public static final Pose2d AB_ALGAE = new Pose2d(3.15, 4.02, ReefRotations.AB_ROTATION);
        public static final Pose2d B = new Pose2d(3.15, 3.85, ReefRotations.AB_ROTATION);
        public static final Pose2d C = new Pose2d(3.69, 2.96, ReefRotations.CD_ROTATION);
        public static final Pose2d CD_ALGAE = new Pose2d(3.79, 2.86, ReefRotations.CD_ROTATION);
        public static final Pose2d D = new Pose2d(3.94, 2.79, ReefRotations.CD_ROTATION);
        public static final Pose2d E = new Pose2d(5.01, 2.81, ReefRotations.EF_ROTATION);
        public static final Pose2d EF_ALGAE = new Pose2d(5.14, 2.87, ReefRotations.EF_ROTATION);
        public static final Pose2d F = new Pose2d(5.29, 2.96, ReefRotations.EF_ROTATION);
        public static final Pose2d G = new Pose2d(5.85, 3.87, ReefRotations.GH_ROTATION);
        public static final Pose2d GH_ALGAE = new Pose2d(5.85, 4.02, ReefRotations.GH_ROTATION);
        public static final Pose2d H = new Pose2d(5.85, 4.18, ReefRotations.GH_ROTATION);
        public static final Pose2d I = new Pose2d(5.29, 5.12, ReefRotations.IJ_ROTATION);
        public static final Pose2d IJ_ALGAE = new Pose2d(5.14, 5.19, ReefRotations.IJ_ROTATION);
        public static final Pose2d J = new Pose2d(5.01, 5.29, ReefRotations.IJ_ROTATION);
        public static final Pose2d K = new Pose2d(3.95, 5.29, ReefRotations.KL_ROTATION);
        public static final Pose2d KL_ALGAE = new Pose2d(3.82, 5.19, ReefRotations.KL_ROTATION);
        public static final Pose2d L = new Pose2d(3.65, 5.12, ReefRotations.KL_ROTATION);
        public static final ReefSide[] SIDES = new ReefSide[]
        { new ReefSide(A, AB_ALGAE, B), new ReefSide(C, CD_ALGAE, D), new ReefSide(E, EF_ALGAE, F),
                new ReefSide(G, GH_ALGAE, H), new ReefSide(I, IJ_ALGAE, J), new ReefSide(K, KL_ALGAE, L) };

        public static final Pose2d[] REEF_POSES_CIRCLE = new Pose2d[]
        { A, B, C, D, E, F, G, H, I, J, K, L };

        public static Pose2d getNextClockwisePose(Pose2d pose)
        {
            SmartDashboard.putString("Received Pose", pose.toString());
            for (int i = 0; i < REEF_POSES_CIRCLE.length; i++)
            {
                if ((int) (REEF_POSES_CIRCLE[i].getTranslation().getDistance(pose.getTranslation()) * 1000) == 0
                        && (double) REEF_POSES_CIRCLE[i].getRotation().getDegrees() == (double) pose.getRotation()
                                .getDegrees())
                {
                    SmartDashboard.putNumber("Current Distance",
                            REEF_POSES_CIRCLE[i].getTranslation().getDistance(pose.getTranslation()));
                    SmartDashboard.putNumber("Current Rotation", REEF_POSES_CIRCLE[i].getRotation().getDegrees());
                    return REEF_POSES_CIRCLE[(i + 1) % REEF_POSES_CIRCLE.length];
                }
            }
            return pose;
        }

        public static Pose2d getNextCounterClockwisePose(Pose2d pose)
        {
            SmartDashboard.putString("Received Other Pose", pose.toString());

            for (int i = 0; i < REEF_POSES_CIRCLE.length; i++)
            {
                if ((int) (REEF_POSES_CIRCLE[i].getTranslation().getDistance(pose.getTranslation()) * 1000) == 0
                        && (double) REEF_POSES_CIRCLE[i].getRotation().getDegrees() == (double) pose.getRotation()
                                .getDegrees())
                {
                    SmartDashboard.putNumber("Current Distance",
                            REEF_POSES_CIRCLE[i].getTranslation().getDistance(pose.getTranslation()));
                    SmartDashboard.putNumber("Current Rotation", REEF_POSES_CIRCLE[i].getRotation().getDegrees());

                    return REEF_POSES_CIRCLE[(i - 1 + REEF_POSES_CIRCLE.length) % REEF_POSES_CIRCLE.length];
                }
            }
            return pose;
        }
    }

    public static final HashMap<ReefLocations, Pose2d> REEF_POSITIONS = new HashMap<>();
    static
    {
        REEF_POSITIONS.put(ReefLocations.A, ReefPositions.A);
        REEF_POSITIONS.put(ReefLocations.B, ReefPositions.B);
        REEF_POSITIONS.put(ReefLocations.C, ReefPositions.C);
        REEF_POSITIONS.put(ReefLocations.D, ReefPositions.D);
        REEF_POSITIONS.put(ReefLocations.E, ReefPositions.E);
        REEF_POSITIONS.put(ReefLocations.F, ReefPositions.F);
        REEF_POSITIONS.put(ReefLocations.G, ReefPositions.G);
        REEF_POSITIONS.put(ReefLocations.H, ReefPositions.H);
        REEF_POSITIONS.put(ReefLocations.I, ReefPositions.I);
        REEF_POSITIONS.put(ReefLocations.J, ReefPositions.J);
        REEF_POSITIONS.put(ReefLocations.K, ReefPositions.K);
        REEF_POSITIONS.put(ReefLocations.L, ReefPositions.L);
        REEF_POSITIONS.put(ReefLocations.AB_ALGAE, ReefPositions.AB_ALGAE);
        REEF_POSITIONS.put(ReefLocations.CD_ALGAE, ReefPositions.CD_ALGAE);
        REEF_POSITIONS.put(ReefLocations.EF_ALGAE, ReefPositions.EF_ALGAE);
        REEF_POSITIONS.put(ReefLocations.GH_ALGAE, ReefPositions.GH_ALGAE);
        REEF_POSITIONS.put(ReefLocations.IJ_ALGAE, ReefPositions.IJ_ALGAE);
        REEF_POSITIONS.put(ReefLocations.KL_ALGAE, ReefPositions.KL_ALGAE);
        REEF_POSITIONS.put(ReefLocations.LEFT_CORAL_STATION, CoralStations.LEFT);
        REEF_POSITIONS.put(ReefLocations.RIGHT_CORAL_STATION, CoralStations.RIGHT);
        REEF_POSITIONS.put(ReefLocations.PROCESSOR_STATION, ProcessorStations.PROCESSOR_STATION);
    }

    /**
     * All poses are BLUE relative, and left and right are from the perspective of
     * the drive station
     */
    public static class CoralStations
    {
        public static final Pose2d LEFT = new Pose2d(1.18, 7.07, Rotation2d.fromDegrees(307.5));
        public static final Pose2d RIGHT = new Pose2d(1.11, 1.00, Rotation2d.fromDegrees(52.5));
    }

    /**
     * All poses are BLUE relative
     */
    public static class ProcessorStations
    {
        public static final Pose2d PROCESSOR_STATION = new Pose2d(6.34, 0.44, Rotation2d.fromDegrees(270));
    }

    /**
     * Gets a rotation based on the alliance
     * 
     * @param alliance The alliance color
     * @return The rotation (0 for blue, 180 for red)
     */
    public static Rotation2d getFieldRotation(Alliance alliance)
    {
        if (alliance == Alliance.Blue)
        {
            return Rotation2d.fromDegrees(0);
        } else
        {
            return Rotation2d.fromDegrees(180);
        }
    }

    public static Pose2d convertPoseByAlliance(Pose2d pose, Alliance alliance)
    {
        if (alliance == Alliance.Blue)
        {
            return pose;
        } else
        {
            return new Pose2d(FIELD_LENGTH.in(Meters) - pose.getX(), FIELD_WIDTH.in(Meters) - pose.getY(),
                    pose.getRotation().plus(Rotation2d.fromDegrees(180)));
        }
    }

    public static Pose2d convertPoseByAlliance(Pose2d pose)
    {
        return convertPoseByAlliance(pose, getAlliance());
    }

    public static Translation2d convertTranslationByAlliance(Translation2d pose, Alliance alliance)
    {
        if (alliance == Alliance.Blue)
        {
            return pose;
        } else
        {
            return new Translation2d(FIELD_LENGTH.in(Meters) - pose.getX(), FIELD_WIDTH.in(Meters) - pose.getY());
        }
    }

    public static Alliance getAlliance()
    {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static class CoralRotations
    {
        public static final Rotation2d BLUE_LEFT = Rotation2d.fromDegrees(120);
        public static final Rotation2d BLUE_RIGHT = Rotation2d.fromDegrees(240);
        public static final Rotation2d RED_LEFT = Rotation2d.fromDegrees(60);
        public static final Rotation2d RED_RIGHT = Rotation2d.fromDegrees(300);
    }

    public static boolean isAtCoralRotation(Rotation2d rotation)
    {
        double degrees = rotation.getDegrees();
        degrees = MathUtil.inputModulus(degrees, 0, 360);
        if (getAlliance() == Alliance.Red)
        {
            return Math.abs(degrees - CoralRotations.RED_LEFT.getDegrees()) <= 10
                    || Math.abs(degrees - CoralRotations.RED_RIGHT.getDegrees()) <= 10;
        } else
        {
            return Math.abs(degrees - CoralRotations.BLUE_LEFT.getDegrees()) <= 10
                    || Math.abs(degrees - CoralRotations.BLUE_RIGHT.getDegrees()) <= 10;
        }
    }
}
