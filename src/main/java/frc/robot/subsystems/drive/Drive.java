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
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.TunerConstants;
import frc.robot.zippy.constants.ZippyConstants;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase
{

	static final Lock odometryLock = new ReentrantLock();
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final Module[] modules;
	private final SysIdRoutine sysId;
	private final TunerConstants tunerConstants;
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
			double trackWidthX, double trackWidthY, TunerConstants tunerConstants, ModuleIO... moduleIOs)
	{

		this.gyroIO = gyroIO;
		this.tunerConstants = tunerConstants;
		modules = new Module[moduleIOs.length];

		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) ->
		{
			Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) ->
		{
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});

		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(null, null, null,
						(state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
				new SysIdRoutine.Mechanism((voltage) ->
				{
					for (int i = 0; i < moduleIOs.length; i++)
					{
						modules[i] = new Module(moduleIOs[i], i, tunerConstants.getConstantsAtPos(i));
						modules[i].runCharacterization(voltage.in(Volts));
					}
				}, null, this));

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> runVelocity(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            ZippyConstants.PathplannerConstants.robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
	}

	public void periodic()
	{
		odometryLock.lock();
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

		if (DriverStation.isDisabled())
		{
			for (var module : modules)
			{
				module.stop();
			}
		}
		if (DriverStation.isDisabled())
		{
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]
			{});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]
			{});
		}

		double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
		int sampleCount = sampleTimestamps.length;
		for (int i = 0; i < sampleCount; i++)
		{
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
			SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
			for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++)
			{
				modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
				moduleDeltas[moduleIndex] = new SwerveModulePosition(
						modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
						modulePositions[moduleIndex].angle);
				lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
			}

			if (gyroInputs.connected)
			{
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else
			{
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			}

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
		speeds.discretize(0.02);
		SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, tunerConstants.kSpeedAt12Volts());

		Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
		Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

		for (int i = 0; i < modules.length; i++)
		{
			modules[i].runSetpoint(setpointStates[i]);
		}

		Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
	}

	/**
	 * Runs the drive at the desired velocity.
	 * 
	 * @param output the output to run the drive at
	 */
	public void runCharacterization(double output)
	{
		for (var module : modules)
		{
			module.runCharacterization(output);
		}
	}

	/** Stops the drive. */
	public void stop()
	{
		runVelocity(new ChassisSpeeds());
	}

	/**
	 * Stops the drive with the wheels locked in the x direction.
	 */
	public void xLock()
	{
		Rotation2d[] headings = new Rotation2d[modules.length];
		for (int i = 0; i < modules.length; i++)
		{
			headings[i] = getModuleTranslations()[i].getAngle();
		}
		kinematics.resetHeadings(headings);
		stop();
	}

	/**
	 * Returns a command to complete a quasistatic test in the specified direction.
	 * 
	 * @param direction the direction to test
	 * @return a command to run the test
	 * @see frc.robot.subsystems.drive.Drive#sysIdDynamic
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
	{
		return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
	}

	/**
	 * Returns a command to complete test in the specified direction.
	 * 
	 * @param direction the direction to test
	 * @return a command to run the test
	 * @see frc.robot.subsystems.drive.Drive#sysIdQuasistatic
	 */
	public Command sysIdDynamic(SysIdRoutine.Direction direction)
	{
		return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
	}

	/**
	 * Returns the module states (turn angles and drive velocities) for all of the
	 * modules.
	 */
	@AutoLogOutput(key = "SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates()
	{
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++)
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
		SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++)
		{
			states[i] = modules[i].getPosition();
		}
		return states;
	}

	/**
	 * Returns the chassis speeds for the swerve drive kinematics.
	 */
	@AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
	private ChassisSpeeds getChassisSpeeds()
	{
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	/**
	 * Returns the wheel radius characterization position for all of the modules.
	 */

	public double[] getWheelRadiusCharacterizationPosition()
	{
		double[] values = new double[modules.length];
		for (int i = 0; i < modules.length; i++)
		{
			values[i] = modules[i].getWheelRadiusCharacterizationPosition();
		}
		return values;
	}

	/**
	 * Returns the feedforward characterization velocity for all of the modules.
	 */
	public double getFFCharacterizationVelocity()
	{
		double output = 0.0;
		for (int i = 0; i < modules.length; i++)
		{
			output += modules[i].getFFCharacterizationVelocity() / 4.0; // TODO: do I replace 4.0???
		}
		return output;
	}

	/**
	 * Returns the current odometry pose.
	 * 
	 * @return a Pose2d of the current odometry pose
	 */
	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d getPose()
	{
		return poseEstimator.getEstimatedPosition();
	}

    public void resetPose(Pose2d pose)
    {
        poseEstimator.resetPose(pose);
    }

	/**
	 * Returns the current odometry pose.
	 * 
	 * @return a Rotation2d of the current odometry pose
	 * @see frc.robot.subsystems.drive.Drive#getPose
	 */
	public Rotation2d getRotation()
	{
		return getPose().getRotation();
	}

	/**
	 * Sets the current odometry pose.
	 * 
	 * @param pose the pose to set
	 * @see frc.robot.subsystems.drive.Drive#getPose
	 */
	public void setPose(Pose2d pose)
	{
		poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
	}

	/**
	 * Adds a vision measurement to the pose estimator.
	 * 
	 * @param visionRobotPoseMeters    the robot pose from the vision measurement
	 * @param timestampSeconds         the timestamp of the vision measurement
	 * @param visionMeasurementStdDevs the standard deviations of the vision
	 */
	public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs)
	{
		poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
	}

	/**
	 * Gets the max linear speed in meters per second.
	 * 
	 * @return the max linear speed
	 */
	public double getMaxLinearSpeedMetersPerSec()
	{
		return tunerConstants.kSpeedAt12Volts().in(MetersPerSecond);
	}

	/**
	 * Returns the maximum angular speed in radians per second.
	 * 
	 * @return the maximum angular speed
	 */
	public double getMaxAngularSpeedRadPerSec()
	{
		return tunerConstants.kSpeedAt12Volts().in(MetersPerSecond);
	}

	/**
	 * Returns the module translations for the swerve drive kinematics.
	 * 
	 * @return the module translations
	 */
	public Translation2d[] getModuleTranslations()
	{
		return new Translation2d[]
		{ new Translation2d(tunerConstants.getConstantsAtPos(0).LocationX,
				tunerConstants.getConstantsAtPos(0).LocationY),
				new Translation2d(tunerConstants.getConstantsAtPos(1).LocationX,
						tunerConstants.getConstantsAtPos(1).LocationY),
				new Translation2d(tunerConstants.getConstantsAtPos(2).LocationX,
						tunerConstants.getConstantsAtPos(2).LocationY),
				new Translation2d(tunerConstants.getConstantsAtPos(3).LocationX,
						tunerConstants.getConstantsAtPos(3).LocationY) };
	}

	/**
	 * This sets the rotation of the robot to zero relative to the field.
	 */
	public void resetOrientation()
	{
		poseEstimator.resetPosition(rawGyroRotation, getModulePositions(),
				new Pose2d(getPose().getTranslation(), new Rotation2d()));
	}

    /**
     * Creates a new path using the pose and assigns the AutoBuilder to follow that path
     */
    public PathPlannerPath createPathToPose(Pose2d pose) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            pose
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90))
        );
        path.preventFlipping = true;

        return path;
    }
}
