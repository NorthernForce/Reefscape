package frc.robot.subsystems.phoenix6;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;

public class PhoenixCommandDrive extends TunerSwerveDrivetrain implements Subsystem
{
    @SuppressWarnings("unused")
    private final LinearVelocity maxSpeed;
    @SuppressWarnings("unused")
    private final AngularVelocity maxAngularSpeed;
    private final Alert motorDisconnectedAlert;
    private final Alert encoderDisconnectedAlert;
    private ArrayList<Integer> disconnectedMotorArray;
    private ArrayList<Integer> disconnectedEncoderArray;
    private String motorAlertString = "";
    private String encoderAlertString = "";
    private Supplier<Distance> safeDriveSupplier;
    private final Distance safeDriveDistance;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final PIDController xPid;
    private final PIDController yPid;
    private final PIDController rPid;
    private final AutoFactory factory;

    /**
     * Create a new PhoenixCommandDrive
     * 
     * @param drivetrainConstants the drivetrain constants
     * @param maxSpeed            the maximum speed of the robot linearly
     * @param maxAngularSpeed     the maximum speed of the robot rotationally
     * @param moduleConstants     the module constants
     */
    private PhoenixCommandDrive(SwerveDrivetrainConstants drivetrainConstants, LinearVelocity maxSpeed,
            AngularVelocity maxAngularSpeed, Distance safeDriveDistance, PIDController xPid, PIDController yPid,
            PIDController rPid, SwerveModuleConstants<?, ?, ?>... moduleConstants)
    {
        super(drivetrainConstants, moduleConstants);
        CommandScheduler.getInstance().registerSubsystem(this);
        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
        motorDisconnectedAlert = new Alert("", Alert.AlertType.kWarning);
        encoderDisconnectedAlert = new Alert("", Alert.AlertType.kWarning);
        disconnectedMotorArray = new ArrayList<>();
        disconnectedEncoderArray = new ArrayList<>();
        safeDriveSupplier = null;
        this.safeDriveDistance = safeDriveDistance;
        this.xPid = xPid;
        this.yPid = yPid;
        this.rPid = rPid;
        factory = new AutoFactory(this::getPose, this::resetPose, (SwerveSample sample) ->
        {
            var pose = getPose();
            var speeds = new ChassisSpeeds(sample.vx + xPid.calculate(pose.getX(), sample.x),
                    sample.vy + yPid.calculate(pose.getY(), sample.y),
                    sample.omega + rPid.calculate(pose.getRotation().getRadians(), sample.heading));
            runVelocity(speeds);
        }, true, this);
        factory.newRoutine("routine");
    }

    public PhoenixCommandDrive(SwerveDrivetrainConstants drivetrainConstants, LinearVelocity maxSpeed,
            AngularVelocity maxAngularSpeed, Distance safeDriveDistance, PIDController xPid, PIDController yPid,
            PIDController rPid, Angle[] moduleOffsets, SwerveModuleConstants<?, ?, ?>... moduleConstants)
    {
        this(drivetrainConstants, maxSpeed, maxAngularSpeed, safeDriveDistance, xPid, yPid, rPid,
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

    /**
     * Get a command that drives the robot by joystick input
     * 
     * @param xSupplier     x input (relative to the field)
     * @param ySupplier     y input (relative to the field)
     * @param omegaSupplier omega input (rotational rate)
     * @return a command that drives the robot by joystick input
     */
    public Command getDriveByJoystickCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier)
    {
        SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric().withDeadband(0.1)
                .withRotationalDeadband(0.1);
        return applyRequest(() ->
        {
            var x = xSupplier.getAsDouble();
            var y = ySupplier.getAsDouble();
            var omega = omegaSupplier.getAsDouble();
            if (safeDriveSupplier != null && safeDriveSupplier.get().lte(safeDriveDistance))
            {
                var robotX = x * getState().Pose.getRotation().getCos() - y * getState().Pose.getRotation().getSin();
                var robotY = x * getState().Pose.getRotation().getSin() + y * getState().Pose.getRotation().getCos();
                robotX = Math.min(robotX, 0);
                x = robotX * getState().Pose.getRotation().getCos() + robotY * getState().Pose.getRotation().getSin();
                y = -robotX * getState().Pose.getRotation().getSin() + robotY * getState().Pose.getRotation().getCos();
            }
            return fieldCentric.withVelocityX(x).withVelocityY(y).withRotationalRate(omega);
        });
    }

    public void enableSafeDrive(Supplier<Distance> distanceSupplier)
    {
        safeDriveSupplier = distanceSupplier;
    }

    public void disableSafeDrive()
    {
        safeDriveSupplier = null;
    }

    /**
     * Get a command that locks the robot in place by point the wheels towards the
     * center of the robot
     * 
     * @return a command that locks the robot in place
     */
    public Command getXLockCommand()
    {
        final var request = new SwerveRequest.SwerveDriveBrake();
        return applyRequest(() -> request);
    }

    /**
     * Lets the swerve drive idle
     * 
     * @return a command that lets the swerve drive idle
     */
    public Command getIdleCommand()
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
    public Command getResetOrientationCommand(Rotation2d orientation)
    {
        return runOnce(() ->
        {
            resetRotation(orientation);
        });
    }

    @Override
    public void simulationPeriodic()
    {
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    @AutoLogOutput
    public Pose2d getPose()
    {
        return getState().Pose;
    }

    @AutoLogOutput
    public SwerveModuleState[] getModuleStates()
    {
        return getState().ModuleStates;
    }

    @AutoLogOutput
    public SwerveModuleState[] getTargetModuleStates()
    {
        return getState().ModuleTargets;
    }

    @AutoLogOutput
    public ChassisSpeeds getChassisSpeeds()
    {
        return getState().Speeds;
    }

    public void runVelocity(ChassisSpeeds speeds)
    {
        setControl(new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity)
                .withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    public void setBrakeMode()
    {
        configNeutralMode(NeutralModeValue.Brake);
    }

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

    public Command getSysIdTranslationQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineTranslation.quasistatic(direction);
    }

    public Command getSysIdTranslationDynamic(SysIdRoutine.Direction direction)
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

    public Command getSysIdSteerQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineSteer.quasistatic(direction);
    }

    public Command getSysIdSteerDynamic(SysIdRoutine.Direction direction)
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

    public Command getSysIdRotationQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineRotation.quasistatic(direction);
    }

    public Command getSysIdRotationDynamic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineRotation.dynamic(direction);
    }

    public Command getFollowPathCommand(String pathName)
    {
        return Commands.sequence(Commands.runOnce(() ->
        {
            xPid.reset();
            yPid.reset();
            rPid.reset();
        }), factory.trajectoryCmd(pathName));
    }

    public Translation2d[] getWaypoints(String pathName)
    {
        var trajectory = factory.newRoutine("routine").trajectory(pathName).getRawTrajectory();
        Translation2d[] waypoints = new Translation2d[trajectory.getPoses().length];
        for (int i = 0; i < trajectory.getPoses().length; i++)
        {
            waypoints[i] = trajectory.getPoses()[i].getTranslation();
        }
        return waypoints;
    }

    public Pose2d getInitialPose(String pathName)
    {
        var trajectory = factory.newRoutine("routine").trajectory(pathName);
        return trajectory.getInitialPose().orElse(new Pose2d(-1, -1, new Rotation2d()));
    }
}