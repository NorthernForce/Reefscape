package frc.robot.subsystems.phoenix6.requests;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CloseDriveToPoseWithRealsenseRequest implements SwerveRequest
{
    private final FieldCentricFacingAngle facingAngle;
    private final Supplier<Pose2d> poseGetter;
    private final PIDController xPID;
    private final PIDController yPID;
    private final Supplier<Optional<Distance>> realsenseDistanceSupplier;
    private final PIDController realsensePID;

    public CloseDriveToPoseWithRealsenseRequest(Pose2d pose, double tP, double tI, double tD, double rP, double rI, double rD,
            Supplier<Pose2d> poseGetter, Supplier<Optional<Distance>> realsenseDistanceSupplier)
    {
        this.xPID = new PIDController(tP, tI, tD);
        this.yPID = new PIDController(tP, tI, tD);
        this.realsensePID = new PIDController(tP, tI, tD);
        xPID.setTolerance(0.05);
        yPID.setTolerance(0.05);
        xPID.setSetpoint(pose.getX());
        yPID.setSetpoint(pose.getY());
        this.facingAngle = new FieldCentricFacingAngle();
        facingAngle.HeadingController.setPID(rP, rI, rD);
        facingAngle.HeadingController.enableContinuousInput(0, Math.PI * 2);
        facingAngle.HeadingController.setTolerance(Math.toRadians(2));
        this.poseGetter = poseGetter;
        facingAngle.withTargetDirection(pose.getRotation());
        facingAngle.withDriveRequestType(DriveRequestType.Velocity);
        this.realsenseDistanceSupplier = realsenseDistanceSupplier;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply)
    {
        double vx = xPID.calculate(poseGetter.get().getX());
        double vy = yPID.calculate(poseGetter.get().getY());
        ChassisSpeeds fieldRelative = new ChassisSpeeds(vx, vy, 0);
        Optional<Distance> realsenseDistance = realsenseDistanceSupplier.get();
        if (realsenseDistance.isPresent())
        {
            var distance = realsenseDistance.get();
            double rD = realsensePID.calculate(distance.in(Meters));
            var robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, parameters.currentPose.getRotation());
            robotRelative.vyMetersPerSecond = rD;
            fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, parameters.currentPose.getRotation());
        }
        facingAngle.withVelocityX(MathUtil.clamp(fieldRelative.vxMetersPerSecond, -1, 1));
        facingAngle.withVelocityY(MathUtil.clamp(fieldRelative.vyMetersPerSecond, -1, 1));
        facingAngle.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
        return facingAngle.apply(parameters, modulesToApply);
    }

    public boolean isFinished()
    {
        if (realsenseDistanceSupplier.get().isPresent())
        {
            return realsensePID.atSetpoint() && facingAngle.HeadingController.atSetpoint();
        }
        return xPID.atSetpoint() && yPID.atSetpoint() && facingAngle.HeadingController.atSetpoint();
    }

}
