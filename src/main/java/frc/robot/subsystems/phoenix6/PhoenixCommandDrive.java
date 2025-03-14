package frc.robot.subsystems.phoenix6;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.phoenix6.requests.CloseDriveToPoseRequest;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;

public class PhoenixCommandDrive extends TunerSwerveDrivetrain implements Subsystem
{
    private final LinearVelocity maxSpeed;
    private final AngularVelocity maxAngularSpeed;
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final Alert motorDisconnectedAlert;
    private final Alert encoderDisconnectedAlert;
    private ArrayList<Integer> disconnectedMotorArray;
    private ArrayList<Integer> disconnectedEncoderArray;
    private String motorAlertString = "";
    private String encoderAlertString = "";
    private final SwerveDrivePoseEstimator poseEstimator;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /**
     * Create a new PhoenixCommandDrive
     * 
     * @param drivetrainConstants the drivetrain constants
     * @param maxSpeed            the maximum speed of the robot linearly
     * @param maxAngularSpeed     the maximum speed of the robot rotationally
     * @param moduleConstants     the module constants
     */
    public PhoenixCommandDrive(SwerveDrivetrainConstants drivetrainConstants, LinearVelocity maxSpeed,
            AngularVelocity maxAngularSpeed, PIDConstants linearPIDConstants, PIDConstants angularPIDConstants,
            SwerveModuleConstants<?, ?, ?>... moduleConstants)
    {
        super(drivetrainConstants, moduleConstants);
        CommandScheduler.getInstance().registerSubsystem(this);
        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
        motorDisconnectedAlert = new Alert("", Alert.AlertType.kWarning);
        encoderDisconnectedAlert = new Alert("", Alert.AlertType.kWarning);
        disconnectedMotorArray = new ArrayList<>();
        disconnectedEncoderArray = new ArrayList<>();
        // Configure the Pathplanner AutoBuilder for easier pathfinding
        configureAutoBuilder(linearPIDConstants, angularPIDConstants);
        poseEstimator = new SwerveDrivePoseEstimator(getKinematics(), getState().RawHeading, getState().ModulePositions,
                new Pose2d());
    }

    @Override
    public void resetPose(Pose2d pose)
    {
        super.resetPose(pose);
        poseEstimator.resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp)
    {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestamp);
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp, Vector<N3> stdDevs)
    {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestamp, stdDevs);
    }

    public Rotation2d getHeading()
    {
        return getState().Pose.getRotation();
    }

    private void configureAutoBuilder(PIDConstants linear, PIDConstants angular)
    {
        try
        {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(this::getPose, this::resetPose, () -> getState().Speeds,
                    (speeds, feedforwards) -> setControl(applyRobotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(new PIDConstants(linear.kP, linear.kI, linear.kD),
                            new PIDConstants(angular.kP, angular.kI, angular.kD)),
                    config, () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this);
        } catch (Exception e)
        {
            e.printStackTrace();
        }

    }

    public PhoenixCommandDrive(SwerveDrivetrainConstants drivetrainConstants, LinearVelocity maxSpeed,
            AngularVelocity maxAngularSpeed, PIDConstants linearPIDConstants, PIDConstants angularPIDConstants,
            Angle[] moduleOffsets, SwerveModuleConstants<?, ?, ?>... moduleConstants)
    {
        this(drivetrainConstants, maxSpeed, maxAngularSpeed, linearPIDConstants, angularPIDConstants,
                new SwerveModuleConstants[]
                { moduleConstants[0].withEncoderOffset(moduleOffsets[0]),
                        moduleConstants[1].withEncoderOffset(moduleOffsets[1]),
                        moduleConstants[2].withEncoderOffset(moduleOffsets[2]),
                        moduleConstants[3].withEncoderOffset(moduleOffsets[3]) });
    }

    /**
     * Apply a request to the drivetrain (runs the request each loop)
     * 
     * @param requestSupplier the request to apply
     * @return a command that applies the request
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier)
    {
        return run(() ->
        {
            setControl(requestSupplier.get());
        });
    }

    public Command driveWithRobotRelativeSpeeds(Supplier<ChassisSpeeds> speedsSupplier)
    {
        SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.Velocity);
        return applyRequest(() ->
        {
            var speeds = speedsSupplier.get();
            return robotCentric.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond);
        });
    }

    public Command driveWithFieldRelativeSpeeds(Supplier<ChassisSpeeds> speedsSupplier)
    {
        SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.Velocity);
        return applyRequest(() ->
        {
            var speeds = speedsSupplier.get();
            return fieldCentric.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond);
        });
    }

    public Command driveWithRobotRelativeDutyCycle(Supplier<ChassisSpeeds> speedsSupplier)
    {
        SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        return applyRequest(() ->
        {
            var speeds = speedsSupplier.get();
            return robotCentric.withVelocityX(speeds.vxMetersPerSecond * maxSpeed.in(MetersPerSecond))
                    .withVelocityY(speeds.vyMetersPerSecond * maxSpeed.in(MetersPerSecond))
                    .withRotationalRate(speeds.omegaRadiansPerSecond * maxAngularSpeed.in(RadiansPerSecond));
        });
    }

    public Command driveWithFieldRelativeDutyCycle(Supplier<ChassisSpeeds> speedsSupplier)
    {
        SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        return applyRequest(() ->
        {
            var speeds = speedsSupplier.get();
            return fieldCentric.withVelocityX(speeds.vxMetersPerSecond * maxSpeed.in(MetersPerSecond))
                    .withVelocityY(speeds.vyMetersPerSecond * maxSpeed.in(MetersPerSecond))
                    .withRotationalRate(speeds.omegaRadiansPerSecond * maxAngularSpeed.in(RadiansPerSecond));
        });
    }

    public Command closeDriveToPose(Pose2d pose)
    {
        CloseDriveToPoseRequest request = new CloseDriveToPoseRequest(pose, 4, 0, 0, 5, 0, 0,
                MetersPerSecond.of(1), Inches.of(1.375), Degrees.of(1.5),
                () -> poseEstimator.getEstimatedPosition());
        Logger.recordOutput("TargetPose", pose);
        return applyRequest(() -> request).until(() -> request.isFinished());
    }

    /**
     * Get a command that drives the robot by joystick input
     * 
     * @param xSupplier     x input (relative to the field)
     * @param ySupplier     y input (relative to the field)
     * @param omegaSupplier omega input (rotational rate)
     * @return a command that drives the robot by joystick input
     */
    public Command driveByJoystick(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier)
    {
        ChassisSpeeds speeds = new ChassisSpeeds();
        return driveWithFieldRelativeDutyCycle(() ->
        {
            speeds.vxMetersPerSecond = xSupplier.getAsDouble();
            speeds.vyMetersPerSecond = ySupplier.getAsDouble();
            speeds.omegaRadiansPerSecond = omegaSupplier.getAsDouble();
            return speeds;
        });
    }

    /**
     * Get a command that drives the robot to the left relative to the robot
     * 
     * @param speed the speed to drive at (duty cycle)
     * @return a command that drives the robot to the left
     */
    public Command goRight(double speed)
    {
        ChassisSpeeds speeds = new ChassisSpeeds();
        speeds.vxMetersPerSecond = speed;
        return driveWithRobotRelativeDutyCycle(() -> speeds);
    }

    /**
     * Get a command that drives the robot to the right relative to the robot
     * 
     * @param speed the speed to drive at (duty cycle)
     * @return a command that drives the robot to the right
     */
    public Command goLeft(double speed)
    {
        ChassisSpeeds speeds = new ChassisSpeeds();
        speeds.vxMetersPerSecond = -speed;
        return driveWithRobotRelativeDutyCycle(() -> speeds);
    }

    /**
     * Get a command that drives the robot forward relative to the robot
     * 
     * @param speed the speed to drive at (duty cycle)
     * @return a command that drives the robot forward
     */
    public Command goForward(double speed)
    {
        ChassisSpeeds speeds = new ChassisSpeeds();
        speeds.vyMetersPerSecond = speed;
        return driveWithRobotRelativeDutyCycle(() -> speeds);
    }

    /**
     * Get a command that drives the robot backward relative to the robot
     * 
     * @param speed the speed to drive at (duty cycle)
     * @return a command that drives the robot backward
     */
    public Command goBackward(double speed)
    {
        ChassisSpeeds speeds = new ChassisSpeeds();
        speeds.vxMetersPerSecond = -speed;
        return driveWithRobotRelativeDutyCycle(() -> speeds);
    }

    /**
     * Get a command that moves the robot to a specific position
     * 
     * @param pose                   the pose to move to
     * @param maxVelocity            the maximum velocity
     * @param maxAcceleration        the maximum acceleration
     * @param maxAngularVelocity     the maximum angular velocity
     * @param maxAngularAcceleration the maximum angular acceleration
     * @return a command that moves the robot to a specific position
     */
    public Command driveToPose(Pose2d pose, LinearVelocity maxVelocity, LinearAcceleration maxAcceleration,
            AngularVelocity maxAngularVelocity, AngularAcceleration maxAngularAcceleration)
    {
        PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity,
                maxAngularAcceleration);
        return AutoBuilder.pathfindToPose(pose, constraints, 0.0);
    }

    /**
     * Get a command that locks the robot in place by point the wheels towards the
     * center of the robot
     * 
     * @return a command that locks the robot in place
     */
    public Command xLock()
    {
        final var request = new SwerveRequest.SwerveDriveBrake();
        return applyRequest(() -> request);
    }

    /**
     * Lets the swerve drive idle
     * 
     * @return a command that lets the swerve drive idle
     */
    public Command idle()
    {
        final var request = new SwerveRequest.Idle();
        return applyRequest(() -> request);
    }

    /**
     * Get a command that resets the orientation of the robot
     * 
     * @param orientation the orientation to reset to
     * @return a command that resets the orientation of the robot
     */
    public Command resetOrientation(Rotation2d orientation)
    {
        poseEstimator.resetRotation(orientation);
        return runOnce(() ->
        {
            resetRotation(orientation);
        });
    }

    @AutoLogOutput
    public Pose2d getStatePose()
    {
        return getState().Pose;
    }

    @Override
    public void simulationPeriodic()
    {
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    /**
     * Get the current pose of the robot
     * 
     * @return the current pose of the robot
     */
    @AutoLogOutput
    public Pose2d getPose()
    {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the states of the modules
     * 
     * @return the states of the modules
     */
    @AutoLogOutput
    public SwerveModuleState[] getModuleStates()
    {
        return getState().ModuleStates;
    }

    /**
     * Get the target states of the modules
     * 
     * @return the target states of the modules
     */
    @AutoLogOutput
    public SwerveModuleState[] getTargetModuleStates()
    {
        return getState().ModuleTargets;
    }

    /**
     * Get the speeds of the robot
     * 
     * @return the speeds of the robot
     */
    @AutoLogOutput
    public ChassisSpeeds getChassisSpeeds()
    {
        return getState().Speeds;
    }

    /**
     * Sets the drive motors to brake mode
     */
    public void setBrakeMode()
    {
        configNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Sets the drive motors to coast mode Why would you ever want to do this?
     */
    public void setCoastMode()
    {
        configNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic()
    {
        disconnectedMotorArray.clear();
        for (SwerveModule<TalonFX, TalonFX, ?> module : getModules())
        {
            if (!module.getDriveMotor().isConnected())
            {
                disconnectedMotorArray.add(module.getDriveMotor().getDeviceID());
            }

            if (!module.getSteerMotor().isConnected())
            {
                disconnectedMotorArray.add(module.getSteerMotor().getDeviceID());
            }

            if (!module.getEncoder().isConnected())
            {
                disconnectedEncoderArray.add(module.getEncoder().getDeviceID());
            }
        }

        if (!disconnectedMotorArray.isEmpty())

        {
            motorAlertString = "The motors with the following IDs are disconnected: "
                    + disconnectedMotorArray.stream().map(String::valueOf).collect(Collectors.joining(", "));

            motorDisconnectedAlert.setText(motorAlertString);
            motorDisconnectedAlert.set(true);
        } else
        {
            motorDisconnectedAlert.set(false);
        }

        if (!disconnectedEncoderArray.isEmpty())
        {
            encoderAlertString = "The encoders with the following IDs are disconnected: "
                    + disconnectedEncoderArray.stream().map(String::valueOf).collect(Collectors.joining(", "));

            encoderDisconnectedAlert.setText(encoderAlertString);
            encoderDisconnectedAlert.set(true);
        } else
        {
            encoderDisconnectedAlert.set(false);
        }
        poseEstimator.update(getState().RawHeading, getState().ModulePositions);
    }

    /**
     * Reset the encoder angle to a target angle
     * 
     * @param moduleIdx   the module index
     * @param targetAngle the target angle
     */
    private Angle resetEncoderAngle(int moduleIdx, Angle targetAngle)
    {
        final var module = getModule(moduleIdx);
        final var currentAngle = Rotations.of(module.getCurrentState().angle.getRotations());
        final var delta = targetAngle.minus(currentAngle);
        final var cancoder = module.getEncoder();
        final var config = new CANcoderConfiguration();
        cancoder.getConfigurator().refresh(config);
        final var currentOffest = Rotations.of(config.MagnetSensor.MagnetOffset);
        var newOffset = currentOffest.plus(delta);
        newOffset = Radians.of(MathUtil.angleModulus(newOffset.in(Radians)));
        config.MagnetSensor.MagnetOffset = newOffset.in(Rotations);
        cancoder.getConfigurator().apply(config);
        return newOffset;
    }

    /**
     * Reset the encoder angles to target angles
     * 
     * @param targetAngles the target angles
     * @return the new offsets
     */
    public Angle[] resetEncoderAngles(Angle[] targetAngles)
    {
        final var newOffsets = new Angle[targetAngles.length];
        for (int i = 0; i < targetAngles.length; i++)
        {
            newOffsets[i] = resetEncoderAngle(i, targetAngles[i]);
        }
        return newOffsets;
    }

    // SysIdRoutine stuff
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default
                                                                                                          // ramp rate
                                                                                                          // (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            Seconds.of(5.0), // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null,
                    this));

    /**
     * SysId routine for characterizing translation. This is used to find PID gains
     * 
     * @param direction the direction to characterize
     * @return a command that characterizes translation
     */
    public Command sysIdTranslationQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineTranslation.quasistatic(direction);
    }

    /**
     * SysId routine for characterizing translation. This is used to find PID gains
     * 
     * @param direction the direction to characterize
     * @return a command that characterizes translation
     */
    public Command sysIdTranslationDynamic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineTranslation.dynamic(direction);
    }

    /**
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default ramp
                                                                                                    // rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            Seconds.of(5.0), // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

    /**
     * SysId routine for characterizing steer. This is used to find PID gains
     * 
     * @param direction the direction to characterize
     * @return a command that characterizes steer
     */
    public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineSteer.quasistatic(direction);
    }

    /**
     * SysId routine for characterizing steer. This is used to find PID gains
     * 
     * @param direction the direction to characterize
     * @return a command that characterizes steer
     */
    public Command sysIdSteerDynamic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineSteer.dynamic(direction);
    }

    /**
     * SysId routine for characterizing rotation. This is used to find PID gains for
     * the FieldCentricFacingAngle HeadingController. See the documentation of
     * SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI), Seconds.of(5.0), // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(output ->
            {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            }, null, this));

    /**
     * SysId routine for characterizing rotation. This is used to find PID gains
     * 
     * @param direction the direction to characterize
     * @return a command that characterizes rotation
     */
    public Command sysIdRotationQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineRotation.quasistatic(direction);
    }

    /**
     * SysId routine for characterizing rotation. This is used to find PID gains
     * 
     * @param direction the direction to characterize
     * @return
     */
    public Command sysIdRotationDynamic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineRotation.dynamic(direction);
    }

    /**
     * Backs up the robot
     * 
     * @param time  the time to back up
     * @param speed the speed to back up at
     * @return a command that backs up the robot
     */
    public Command backup(Time time, double speed)
    {
        return goBackward(speed).withTimeout(time);
    }

    /**
     * Get the current speed of the robot
     * 
     * @return the current speed of the robot (a LinearVelocity)
     */
    public LinearVelocity getSpeed()
    {
        var speeds = getState().Speeds;
        return MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    }
}