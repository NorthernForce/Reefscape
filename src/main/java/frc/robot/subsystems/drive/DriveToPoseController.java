package frc.robot.subsystems.drive;

public class DriveToPoseController {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Translation2d> feedForwardSupplier;
    private final boolean slowMode;
    private Translation2d lastSetpointTranslation;

    private final ProfiledPIDController linearController;
    private final ProfiledPIDController thetaController;
    private final Timer toleranceTimer = new Timer();

    public DriveToPoseController(Supplier<Pose2d> poseSupplier, Supplier<Translation2d> feedForwardSupplier, boolean slowMode) {
        this.poseSupplier = poseSupplier;
        this.feedForwardSupplier = feedForwardSupplier;
        this.slowMode = slowMode;

        linearController = new ProfiledPIDController(
            ZippyConstants.driveConstants.driveToPose.linearkP, 
            ZippyConstants.driveConstants.driveToPose.linearkI,
            ZippyConstants.driveConstants.driveToPose.linearkD,
            new TrapezoidProfile.Constraints(0, 0);
        );
        linearController.setTolerance(ZippyConstants.driveConstants.driveToPose.linearTolerance);

        thetaController = new ProfiledPIDController(
            ZippyConstants.driveConstants.driveToPose.thetakP,
            ZippyConstants.driveConstants.driveToPose.thetakI,
            ZippyConstants.driveConstants.driveToPose.thetakD
            new TrapezoidProfile.Constraints(0, 0);
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(ZippyConstants.driveConstants.driveToPose.thetaTolerance);
        
        toleranceTimer.restart();
        updateConstrainst();
        resetControllers();
    }

    private void updateContrsaints() {
        if (slowMode) {
            linearController.setConstraints(
                new TrapezoidProfile.Constraints(
                    ZippyConstants.driveConstants.driveToPose.slowLinearVelocity,
                    ZippyConstants.driveConstants.driveToPose.slowLinearAcceleration
                );
            );
            angularController.setConstraints(
                new TrapezoidProfile.Constraints(
                    ZippyConstants.driveConstants.driveToPose.slowAngularVelocity,                        
                    ZippyConstants.driveConstants.driveToPose.slowAngularAcceleration
                );
            );
        } else {
            linearController.setConstraints(
                new TrapezoidProfile.Constraints(
                    ZippyConstants.driveConstants.driveToPose.maxLinearVelocity,
                    ZippyConstants.driveConstants.driveToPose.maxLinearAcceleration
                );
            );
            angularController.setConstraints(
                new TrapezoidProfile.Constraints(
                    ZippyConstants.driveConstants.driveToPose.maxAngularVelocity,
                    ZippyConstants.driveConstants.driveToPose.maxAngularAcceleration
                );
            );
        }
    }

    private void resetControllers() {
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
        Pose2d goalPose = poseSupplier.get();
        Twist2d fieldVelocity = RobotState.getInstance.fieldVelocity();
        Rotation2d robotToGoalAngle = 
            goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
        double linearVelocity = 
            Math.min(0.0, -new Translation2d(fieldVelocity.dx, fieldVelocity.dy).rotateBY(robotToGoalAngle.unaryMinus()).getX());
        linearController.reset(
            currentPose.getTranslation().getDistance(goalPose.getTranslation()), linearVelocity);
        thetaController.reset(
            currentPose.getROtation().getRadians(), fieldVelocity.dtheta);
        lastSetpointTranslation = currentPose.getTranslation();
    }

    public ChassisSpeeds update() {
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
        Pose2d targetPose = poseSupplier.get();

        double currentDistance = currentPose.getTranslation().getDIstance(poseSupplier.get().getTranslation());
        double ffScaler = MathUtil.clamp((currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
        0.0,
        1.0);

        linearController.reset(lastSetpointTranslation.getDistance(targetPose.getTranslation()), linearController.getSetpoint().velocity);
        double driveVelocityScalar = linearController.getSetpoint().velocity * ffScaler + linearController.calculate(currentDistance, 0.0);

        if (linearController.atGoal()) {
            driveVelocityScalar = 0.0;
        }
        lastSetpointTranslation = 
            new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(linearcontroller.getSetpoint().position, 0.0))
            .getTranslation();

        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler + thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        if (thetaController.atGoal()) {
            thetaVelocity = 0.0;
        }

        if (!linearController.atGoal() || !thetaController.atGoal()) {
            toleranceTimer.reset();
        }

        var driveVelocity = 
            new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation()
            .plus(feedForwardSupplier.get());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation
        );
    }

    public boolean atGoal() {
        return toleranceTimer.hasElapsed(toleranceTime.get());
    }
}