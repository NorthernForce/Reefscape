package frc.robot.subsystems.phoenix6.requests;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CloseDriveToPoseRequest implements SwerveRequest
{
    private final ProfiledPIDController xPID;
    private final ProfiledPIDController yPID;
    private final FieldCentricFacingAngle facingAngle;
    private final Supplier<Pose2d> poseGetter;

    public CloseDriveToPoseRequest(Pose2d pose, double tP, double tI, double tD, double rP, double rI, double rD,
            Supplier<Pose2d> poseGetter)
    {
        this.xPID = new ProfiledPIDController(tP, tI, tD, new Constraints(2, 2));
        this.yPID = new ProfiledPIDController(tP, tI, tD, new Constraints(2, 2));
        xPID.setGoal(pose.getX());
        yPID.setGoal(pose.getY());
        this.facingAngle = new FieldCentricFacingAngle();
        facingAngle.HeadingController.setPID(rP, rI, rD);
        facingAngle.HeadingController.enableContinuousInput(0, Math.PI * 2);
        this.poseGetter = poseGetter;
        facingAngle.withTargetDirection(pose.getRotation());
        facingAngle.withDriveRequestType(DriveRequestType.Velocity);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply)
    {
        double vx = xPID.calculate(poseGetter.get().getX());
        double vy = yPID.calculate(poseGetter.get().getY());
        facingAngle.withVelocityX(vx);
        facingAngle.withVelocityY(vy);
        return facingAngle.apply(parameters, modulesToApply);
    }

}
