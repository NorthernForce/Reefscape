package frc.robot.subsystems.phoenix6;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.phoenix6.requests.XLockRequest;
import frc.robot.zippy.constants.ZippyConstants;

public class PhoenixCommandDrive extends SwerveDrivetrain implements Subsystem
{
	private final LinearVelocity maxSpeed;
	private final AngularVelocity maxAngularSpeed;

	/**
	 * Create a new PhoenixCommandDrive
	 * 
	 * @param drivetrainConstants the drivetrain constants
	 * @param maxSpeed            the maximum speed of the robot linearly
	 * @param maxAngularSpeed     the maximum speed of the robot rotationally
	 * @param moduleConstants     the module constants
	 */
	public PhoenixCommandDrive(SwerveDrivetrainConstants drivetrainConstants, LinearVelocity maxSpeed,
			AngularVelocity maxAngularSpeed, SwerveModuleConstants... moduleConstants)
	{
		super(drivetrainConstants, moduleConstants);
		CommandScheduler.getInstance().registerSubsystem(this);
		this.maxSpeed = maxSpeed;
		this.maxAngularSpeed = maxAngularSpeed;

        // Configure the Pathplanner AutoBuilder for easier pathfinding
		AutoBuilder.configure(this::getPose, this::resetPose, this::getChassisSpeeds,
				(speeds, feedforwards) -> runVelocity(speeds),
				new PPHolonomicDriveController(new PIDConstants(ZippyConstants.PathplannerConstants.linearkP,
						ZippyConstants.PathplannerConstants.linearkI, ZippyConstants.PathplannerConstants.linearkD),
						new PIDConstants(ZippyConstants.PathplannerConstants.angularkP,
								ZippyConstants.PathplannerConstants.angularkI,
								ZippyConstants.PathplannerConstants.angularkD)),
				ZippyConstants.PathplannerConstants.robotConfig, () ->
				{
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent())
					{
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				}, this);
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
     * Get a command that drives the robot at chassis speeds
     * 
     * @param speeds the speeds that the chassis should drive at
     * @return the command to drive the robot at the specified chassis speeds
     */
	public Command runVelocity(ChassisSpeeds speeds)
	{
		return applyRequest(() ->
		{
			return new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds);
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