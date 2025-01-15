package frc.robot.subsystems.phoenix6;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.phoenix6.requests.XLockRequest;

public class PhoenixCommandDrive extends TunerSwerveDrivetrain implements Subsystem
{
	private final LinearVelocity maxSpeed;
	private final AngularVelocity maxAngularSpeed;
	private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

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

		// Configure the Pathplanner AutoBuilder for easier pathfinding
		configureAutoBuilder(linearPIDConstants, angularPIDConstants);
	}

	private void configureAutoBuilder(PIDConstants linear, PIDConstants angular)
	{
		try
		{
			RobotConfig config = RobotConfig.fromGUISettings();
			AutoBuilder.configure(() -> getState().Pose, this::resetPose, () -> getState().Speeds,
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
	 * Get a command to drive the robot to a pose on the field using Pathplanner
	 * 
	 * @param pose The pose that the robot should drive to
	 * @return A command that drives the robot to the specified pose
	 */
	public Command driveToPose(Pose2d pose)
	{
		PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
		return AutoBuilder.pathfindToPose(pose, constraints, 0.0);
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