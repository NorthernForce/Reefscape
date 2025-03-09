package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

public class DriveToPoseRequest implements SwerveRequest
{

    private Pose2d targetPose;
    private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle();
    private final Distance totalDistance;
    private final Rotation2d totalAngle;
    private final LinearVelocity maxSpeed;

    // Create separate controllers for X and Y
    private final ProfiledPIDController xController;

    private final ProfiledPIDController yController;

    private final PIDController postCalculator;

    public DriveToPoseRequest(double kP, double kI, double kD, double postP, double postI, double postD,
            Constraints kConstraints, double kPRotation, double rotationContinuous, Rotation2d totalAngle,
            Distance totalDistance, LinearVelocity maxSpeed)
    {
        xController = new ProfiledPIDController(kP, kI, kD, kConstraints);
        yController = new ProfiledPIDController(kP, kI, kD, kConstraints);
        facingAngle.HeadingController.setP(kPRotation);
        facingAngle.HeadingController.enableContinuousInput(0, rotationContinuous);
        facingAngle.HeadingController.setTolerance(totalAngle.getRadians());

        postCalculator = new PIDController(postP, postI, postD);

        this.totalDistance = totalDistance;
        this.totalAngle = totalAngle;
        this.maxSpeed = maxSpeed;
    }

    public DriveToPoseRequest withPose(Pose2d pose)
    {
        this.targetPose = pose;
        return this;
    }

    private Optional<Distance> postOffset = Optional.empty();

    public DriveToPoseRequest withPostOffset(Optional<Distance> postOffset)
    {
        this.postOffset = postOffset;
        return this;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply)
    {
        Pose2d currentPose = parameters.currentPose;

        // 1) Check distance & heading error
        double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        Rotation2d headingError = targetPose.getRotation().minus(currentPose.getRotation());

        if (distanceError < totalDistance.in(Meters) && Math.abs(headingError.getRadians()) < totalAngle.getRadians())
        {
            // We are close enough: apply Idle request
            return new Idle().apply(parameters, modulesToApply);
        }

        // 2) Update setpoints
        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());

        // 3) Calculate the output velocities
        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());

        // 4) Clamp speeds if desired (avoid extreme overshoot)
        xSpeed = Math.max(xSpeed, Math.min(-maxSpeed.in(MetersPerSecond), maxSpeed.in(MetersPerSecond)));
        ySpeed = Math.max(ySpeed, Math.min(-maxSpeed.in(MetersPerSecond), maxSpeed.in(MetersPerSecond)));

        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
        var robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, parameters.currentPose.getRotation());

        if (postOffset.isPresent())
        {
            robotSpeeds.vxMetersPerSecond = postCalculator.calculate(postOffset.get().in(Meters));
        }

        fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, parameters.currentPose.getRotation());

        // 5) Build the request using FieldCentricFacingAngle
        return facingAngle
                // Flip if needed for driver station perspective
                .withVelocityX(fieldSpeeds.vxMetersPerSecond).withVelocityY(fieldSpeeds.vyMetersPerSecond)
                // We want to face the target heading
                .withTargetDirection(Rotation2d.fromRadians(targetPose.getRotation().getRadians()))
                .apply(parameters, modulesToApply);
    }
}