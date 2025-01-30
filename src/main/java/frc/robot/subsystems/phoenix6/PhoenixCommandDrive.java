package frc.robot.subsystems.phoenix6;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.phoenix6.requests.XLockRequest;

public class PhoenixCommandDrive extends TunerSwerveDrivetrain implements Subsystem
{
	private final LinearVelocity maxSpeed;
	private final AngularVelocity maxAngularSpeed;

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
			AngularVelocity maxAngularSpeed, SwerveModuleConstants<?, ?, ?>... moduleConstants)
	{
		super(drivetrainConstants, moduleConstants);
		CommandScheduler.getInstance().registerSubsystem(this);
		this.maxSpeed = maxSpeed;
		this.maxAngularSpeed = maxAngularSpeed;
	}

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    public Command getSysIdTranslationQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineTranslation.quasistatic(direction);
    }

    public Command getSysIdTranslationDynamic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineTranslation.dynamic(direction);
    }

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    public Command getSysIdSteerQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineSteer.quasistatic(direction);
    }

    public Command getSysIdSteerDynamic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineSteer.dynamic(direction);
    }

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    public Command getSysIdRotationQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineRotation.quasistatic(direction);
    }

    public Command getSysIdRotationDynamic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineRotation.dynamic(direction);
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
}