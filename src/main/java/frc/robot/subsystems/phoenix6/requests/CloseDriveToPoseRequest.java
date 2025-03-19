package frc.robot.subsystems.phoenix6.requests;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CloseDriveToPoseRequest implements SwerveRequest
{
    private final FieldCentricFacingAngle facingAngle;
    private final Supplier<Pose2d> poseGetter;
    private final ProfiledPIDController xPID;
    private final ProfiledPIDController yPID;
    private final LinearVelocity maxVelocity;

    public CloseDriveToPoseRequest(Pose2d pose, double tP, double tI, double tD, double rP, double rI, double rD,
            LinearVelocity maxVelocity, LinearAcceleration maxAcceleration, Supplier<Pose2d> poseGetter)
    {
        this.xPID = new ProfiledPIDController(tP, tI, tD,
                new Constraints(maxVelocity.in(MetersPerSecond), maxAcceleration.in(MetersPerSecondPerSecond)));
        this.yPID = new ProfiledPIDController(tP, tI, tD,
                new Constraints(maxVelocity.in(MetersPerSecond), maxAcceleration.in(MetersPerSecondPerSecond)));
        xPID.setTolerance(0.03);
        yPID.setTolerance(0.03);
        xPID.reset(poseGetter.get().getX());
        yPID.reset(poseGetter.get().getY());
        xPID.setGoal(pose.getX());
        yPID.setGoal(pose.getY());
        this.facingAngle = new FieldCentricFacingAngle();
        facingAngle.HeadingController.setPID(rP, rI, rD);
        facingAngle.HeadingController.enableContinuousInput(0, Math.PI * 2);
        facingAngle.HeadingController.setTolerance(Math.toRadians(2));
        this.poseGetter = poseGetter;
        facingAngle.withTargetDirection(pose.getRotation());
        facingAngle.withDriveRequestType(DriveRequestType.Velocity);
        facingAngle.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
        this.maxVelocity = maxVelocity;
    }

    public void reset()
    {
        xPID.reset(poseGetter.get().getX());
        yPID.reset(poseGetter.get().getY());
        facingAngle.HeadingController.reset();
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply)
    {
        double x = xPID.calculate(poseGetter.get().getX());
        double vx = xPID.getSetpoint().velocity + x;
        double y = yPID.calculate(poseGetter.get().getY());
        double vy = yPID.getSetpoint().velocity + y;
        facingAngle
                .withVelocityX(MathUtil.clamp(vx, -maxVelocity.in(MetersPerSecond), maxVelocity.in(MetersPerSecond)));
        facingAngle
                .withVelocityY(MathUtil.clamp(vy, -maxVelocity.in(MetersPerSecond), maxVelocity.in(MetersPerSecond)));
        return facingAngle.apply(parameters, modulesToApply);
    }

    public boolean isFinished()
    {
        return xPID.atSetpoint() && yPID.atSetpoint() && facingAngle.HeadingController.atSetpoint();
    }

}
