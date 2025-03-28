package frc.robot.subsystems.phoenix6.requests;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.viewer.Viewer.ViewerTarget;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CloseDriveToPoseRequest implements SwerveRequest
{
    private final FieldCentricFacingAngle facingAngle;
    private final Supplier<Pose2d> poseGetter;
    private final PIDController xPID;
    private final PIDController yPID;
    private final PIDController viewerXPID;
    private final PIDController viewerYPID;
    private final LinearVelocity maxVelocity;
    private final Supplier<Optional<ViewerTarget>> viewerTargetSupplier;
    private final Pose2d targetPose;

    public CloseDriveToPoseRequest(Pose2d pose, double tP, double tI, double tD, double rP, double rI, double rD,
            LinearVelocity maxVelocity, Supplier<Pose2d> poseGetter,
            Supplier<Optional<ViewerTarget>> viewerTargetSupplier)
    {
        this.xPID = new PIDController(tP, tI, tD);
        this.yPID = new PIDController(tP, tI, tD);
        this.viewerXPID = new PIDController(tP, tI, tD);
        this.viewerYPID = new PIDController(2, tI, tD);
        xPID.setTolerance(0.02);
        yPID.setTolerance(0.02);
        viewerXPID.setTolerance(0.02);
        viewerYPID.setTolerance(0.02);
        viewerYPID.setSetpoint(0);
        xPID.setSetpoint(pose.getX());
        yPID.setSetpoint(pose.getY());
        this.facingAngle = new FieldCentricFacingAngle();
        facingAngle.HeadingController.setPID(rP, rI, rD);
        facingAngle.HeadingController.enableContinuousInput(0, Math.PI * 2);
        facingAngle.HeadingController.setTolerance(Math.toRadians(2));
        this.poseGetter = poseGetter;
        facingAngle.withTargetDirection(pose.getRotation());
        facingAngle.withDriveRequestType(DriveRequestType.Velocity);
        this.maxVelocity = maxVelocity;
        this.viewerTargetSupplier = viewerTargetSupplier;
        this.targetPose = pose;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply)
    {
        double vx = xPID.calculate(poseGetter.get().getX());
        double vy = yPID.calculate(poseGetter.get().getY());
        ChassisSpeeds targetSpeeds = new ChassisSpeeds(vx, vy, 0);
        Optional<ViewerTarget> target = viewerTargetSupplier.get();
        // if (target.isPresent() &&
        // poseGetter.get().getTranslation().getDistance(targetPose.getTranslation()) <
        // 0.3)
        // {
        // var target_ = target.get();
        // if (!target_.xDistance().isNaN())
        // {
        // double viewerVy = -viewerYPID.calculate(target.get().xDistance());
        // ChassisSpeeds robotRel = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds,
        // parameters.currentPose.getRotation());
        // robotRel.vyMetersPerSecond = viewerVy;
        // targetSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRel,
        // parameters.currentPose.getRotation());
        // Logger.recordOutput("robotRel_vyMetersPerSecond",
        // robotRel.vyMetersPerSecond);
        // System.out.println("PIDing: " + viewerVy);
        // }
        // }
        facingAngle.withVelocityX(MathUtil.clamp(targetSpeeds.vxMetersPerSecond, -maxVelocity.in(MetersPerSecond),
                maxVelocity.in(MetersPerSecond)));
        facingAngle.withVelocityY(MathUtil.clamp(targetSpeeds.vyMetersPerSecond, -maxVelocity.in(MetersPerSecond),
                maxVelocity.in(MetersPerSecond)));
        facingAngle.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
        return facingAngle.apply(parameters, modulesToApply);
    }

    public boolean isFinished()
    {
        return xPID.atSetpoint() && yPID.atSetpoint() && facingAngle.HeadingController.atSetpoint();
        // && (viewerTargetSupplier.get().isEmpty() || viewerYPID.atSetpoint());
    }

}
