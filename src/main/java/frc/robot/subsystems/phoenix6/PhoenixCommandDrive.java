package frc.robot.subsystems.phoenix6;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.phoenix6.requests.XLockRequest;

public class PhoenixCommandDrive extends TunerSwerveDrivetrain implements Subsystem
{
    private final LinearVelocity maxSpeed;
    private final AngularVelocity maxAngularSpeed;
    private final Alert motorDisconnectedAlert;
    private final Alert encoderDisconnectedAlert;
    private ArrayList<Integer> disconnectedMotorArray;
    private ArrayList<Integer> disconnectedEncoderArray;
    private String motorAlertString = "";
    private String encoderAlertString = "";

    /**
     * Create a new PhoenixCommandDrive
     * 
     * @param drivetrainConstants the drivetrain constants
     * @param maxSpeed            the maximum speed of the robot linearly
     * @param maxAngularSpeed     the maximum speed of the robot rotationally
     * @param moduleConstants     the module constants
     */
    public PhoenixCommandDrive(SwerveDrivetrainConstants drivetrainConstants, LinearVelocity maxSpeed,
            AngularVelocity maxAngularSpeed, SwerveModuleConstants<?, ?, ?>... moduleConstants)
    {
        super(drivetrainConstants, moduleConstants);
        CommandScheduler.getInstance().registerSubsystem(this);
        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
        motorDisconnectedAlert = new Alert("", Alert.AlertType.kWarning);
        encoderDisconnectedAlert = new Alert("", Alert.AlertType.kWarning);
        disconnectedMotorArray = new ArrayList<>();
        disconnectedEncoderArray = new ArrayList<>();
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
        SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric().withDeadband(maxSpeed.times(0.1))
                .withRotationalDeadband(maxAngularSpeed.times(0.1));
        return applyRequest(() ->
        {
            return fieldCentric.withVelocityX(maxSpeed.times(xSupplier.getAsDouble()))
                    .withVelocityY(maxSpeed.times(ySupplier.getAsDouble()))
                    .withRotationalRate(maxAngularSpeed.times(omegaSupplier.getAsDouble()));
        });
    }

    /**
     * Get a command that locks the robot in place by point the wheels towards the
     * center of the robot
     * 
     * @return a command that locks the robot in place
     */
    public Command getXLockCommand()
    {
        XLockRequest xLockRequest = new XLockRequest();
        return applyRequest(() -> xLockRequest);
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

}