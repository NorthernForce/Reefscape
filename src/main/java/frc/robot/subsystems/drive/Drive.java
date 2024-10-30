// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase
{

	private final double MAX_LINEAR_SPEED;
	private static double TRACK_WIDTH_X;
	private static double TRACK_WIDTH_Y;
	private final double DRIVE_BASE_RADIUS;
	private final double MAX_ANGULAR_SPEED;
	static final Lock odometryLock = new ReentrantLock();
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final Module[] modules = new Module[4]; // FL, FR, BL, BR
	private final SysIdRoutine sysId;

	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
	private Rotation2d rawGyroRotation = new Rotation2d();
	private SwerveModulePosition[] lastModulePositions = // For delta tracking
			new SwerveModulePosition[]
			{ new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(),
					new SwerveModulePosition() };
	private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
			lastModulePositions, new Pose2d());

	/**
	 * Creates a new Drive object to drive the flipping robot
	 * 
	 * @param gyroIO          the gyro to go by (not used yet I don't think)
	 * @param maxLinearSpeed  the max speed the robot should travel
	 * @param maxAngularSpeed the max speed the robot should turn
	 * @param driveBaseRadius the radius of the base
	 * @param trackWidthX     x delta movement
	 * @param trackWidthY     y delta movement
	 * @param moduleIOs       all the moduleIO for the drive motors. why the flip
	 *                        would we have more than 4? idk ask connor >:(
	 */
	public Drive(GyroIO gyroIO, double maxLinearSpeed, double maxAngularSpeed, double driveBaseRadius,
			double trackWidthX, double trackWidthY, ModuleIO... moduleIOs)
	{
		MAX_ANGULAR_SPEED = maxAngularSpeed;
		MAX_LINEAR_SPEED = maxLinearSpeed;
		TRACK_WIDTH_X = trackWidthX;
		TRACK_WIDTH_Y = trackWidthY;
		DRIVE_BASE_RADIUS = driveBaseRadius;
		this.gyroIO = gyroIO;

		// Start threads (no-op for each if no signals have been created)

		// Configure AutoBuilder for PathPlanner
		AutoBuilder.configureHolonomic(this::getPose, this::setPose,
				() -> kinematics.toChassisSpeeds(getModuleStates()), this::runVelocity,
				new HolonomicPathFollowerConfig(MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
				() -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red,
				this);
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) ->
		{
			Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) ->
		{
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});

		// Configure SysId
		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(null, null, null,
						(state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
				new SysIdRoutine.Mechanism((voltage) ->
				{
					for (int i = 0; i < moduleIOs.length; i++)
					{
						modules[i] = new Module(moduleIOs[i], i);
						modules[i].runCharacterization(voltage.in(Volts));
					}
				}, null, this));
	}

	/**
	 * periodic code to continuosly run
	 */
	public void periodic()
	{
		odometryLock.lock(); // Prevents odometry updates while reading data
		gyroIO.updateInputs(gyroInputs);
		for (var module : modules)
		{
			module.updateInputs();
		}
		odometryLock.unlock();
		Logger.processInputs("Drive/Gyro", gyroInputs);
		for (var module : modules)
		{
			module.periodic();
		}

		// Stop moving when disabled
		if (DriverStation.isDisabled())
		{
			for (var module : modules)
			{
				module.stop();
			}
		}
		// Log empty setpoint states when disabled
		if (DriverStation.isDisabled())
		{
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]
			{});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]
			{});
		}

		// Update odometry
		double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
		int sampleCount = sampleTimestamps.length;
		for (int i = 0; i < sampleCount; i++)
		{
			// Read wheel positions and deltas from each module
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
			SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
			for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++)
			{
				modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
				moduleDeltas[moduleIndex] = new SwerveModulePosition(
						modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
						modulePositions[moduleIndex].angle);
				lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
			}

			// Update gyro angle
			if (gyroInputs.connected)
			{
				// Use the real gyro angle
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else
			{
				// Use the angle delta from the kinematics and module deltas
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			}

			// Apply update
			poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
		}
	}

	/**
	 * Runs the drive at the desired velocity.
	 *
	 * @param speeds Speeds in meters/sec
	 */
	public void runVelocity(ChassisSpeeds speeds)
	{
		// Calculate module setpoints
		ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
		SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

		// Send setpoints to modules
		SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++)
		{
			// The module returns the optimized state, useful for logging
			optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
		}

		// Log setpoint states
		Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
		Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
	}

	/** Stops the drive. */
	public void stop()
	{
		runVelocity(new ChassisSpeeds());
	}

	/**
	 * Stops the drive and turns the modules to an X arrangement to resist movement.
	 * The modules will return to their normal orientations the next time a nonzero
	 * velocity is requested.
	 */
	public void stopWithX()
	{
		Rotation2d[] headings = new Rotation2d[4];
		for (int i = 0; i < 4; i++)
		{
			headings[i] = getModuleTranslations()[i].getAngle();
		}
		kinematics.resetHeadings(headings);
		stop();
	}

	/** Returns a command to run a quasistatic test in the specified direction. */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
	{
		return sysId.quasistatic(direction);
	}

	/** Returns a command to run a dynamic test in the specified direction. */
	public Command sysIdDynamic(SysIdRoutine.Direction direction)
	{
		return sysId.dynamic(direction);
	}

	/**
	 * Returns the module states (turn angles and drive velocities) for all of the
	 * modules.
	 */
	@AutoLogOutput(key = "SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates()
	{
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++)
		{
			states[i] = modules[i].getState();
		}
		return states;
	}

	/**
	 * Returns the module positions (turn angles and drive positions) for all of the
	 * modules.
	 */
	private SwerveModulePosition[] getModulePositions()
	{
		SwerveModulePosition[] states = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++)
		{
			states[i] = modules[i].getPosition();
		}
		return states;
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d getPose()
	{
		return poseEstimator.getEstimatedPosition();
	}

	/** Returns the current odometry rotation. */
	public Rotation2d getRotation()
	{
		return getPose().getRotation();
	}

	/** Resets the current odometry pose. */
	public void setPose(Pose2d pose)
	{
		poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
	}

	/**
	 * Adds a vision measurement to the pose estimator.
	 *
	 * @param visionPose The pose of the robot as measured by the vision camera.
	 * @param timestamp  The timestamp of the vision measurement in seconds.
	 */
	public void addVisionMeasurement(Pose2d visionPose, double timestamp) // TODO: talk to zack
	{
		poseEstimator.addVisionMeasurement(visionPose, timestamp);
	}

	/** Returns the maximum linear speed in meters per sec. */
	public double getMaxLinearSpeedMetersPerSec()
	{
		return MAX_LINEAR_SPEED;
	}

	/** Returns the maximum angular speed in radians per sec. */
	public double getMaxAngularSpeedRadPerSec()
	{
		return MAX_ANGULAR_SPEED;
	}

	/** Returns an array of module translations. */
	public static Translation2d[] getModuleTranslations()
	{
		return new Translation2d[]
		{ new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
				new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
				new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
				new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0) };
	}
}
