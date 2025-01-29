package frc.robot.zippy;

import java.util.Map;
import java.util.function.Supplier;

import org.northernforce.util.NFRRobotContainer;

import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.zippy.constants.ZippyConstants;
import frc.robot.zippy.constants.ZippyTunerConstants;
import frc.robot.zippy.oi.ZippyDriverOI;
import frc.robot.zippy.oi.ZippyOI;
import frc.robot.zippy.oi.ZippyProgrammerOI;

public class ZippyContainer implements NFRRobotContainer
{
	private final PhoenixCommandDrive drive;
	private final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
	private AutoFactory factory = null;
	private AutoChooser autoChooser = null;
	private AutoRoutine hi = null;
	private Command hicmd = null;
	Field2d field;

	public ZippyContainer()
	{
		drive = new PhoenixCommandDrive(ZippyTunerConstants.DrivetrainConstants,
				ZippyConstants.DrivetrainConstants.MAX_SPEED, ZippyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
				ZippyTunerConstants.FrontLeft, ZippyTunerConstants.FrontRight, ZippyTunerConstants.BackLeft,
				ZippyTunerConstants.BackRight);
		drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(alliance));
		field = new Field2d();
		drive.resetPose(new Pose2d(7, 7, new Rotation2d(0)));
		Shuffleboard.getTab("General").add("pose2d", field);
	}

	public PhoenixCommandDrive getDrive()
	{
		return drive;
	}

	@Override
	public void bindOI()
	{
		ZippyOI zippyOI;
		switch (Constants.kOI)
		{
		case PROGRAMMER:
			zippyOI = new ZippyProgrammerOI();
			break;
		case DRIVER:
		default:
			zippyOI = new ZippyDriverOI();
			break;
		}
		zippyOI.bindOI(this);
	}

	@Override
	public void periodic()
	{
		if (alliance != DriverStation.getAlliance().orElse(Alliance.Red))
		{
			drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(alliance));
		}
		field.setRobotPose(drive.getPose());
	}

	@Override
	public void autonomousInit()
	{
		ZippyConstants.AutoConstants.xPID.reset();
		ZippyConstants.AutoConstants.yPID.reset();
		ZippyConstants.AutoConstants.rPID.reset();
		factory = new AutoFactory(drive::getPose, drive::resetPose, (SwerveSample sample) ->
		{
			var robot = drive.getPose();
			var speeds = new ChassisSpeeds(
					sample.vx + ZippyConstants.AutoConstants.xPID.calculate(robot.getX(), sample.x),
					sample.vy + ZippyConstants.AutoConstants.yPID.calculate(robot.getY(), sample.y),
					sample.omega + ZippyConstants.AutoConstants.rPID.calculate(robot.getRotation().getRadians(),
							sample.heading));
			System.out.println("AUTO STATS:");
			System.out.println("x: " + speeds.vxMetersPerSecond);
			System.out.println("y: " + speeds.vyMetersPerSecond);
			System.out.println("r: " + speeds.omegaRadiansPerSecond);
			drive.runVelocity(speeds);
		}, true, drive);

		NamedCommands.registerCommand("test", Commands.runOnce(() -> System.out.println("it works!")));
		var routine = factory.newRoutine("test1");
		var traj = routine.trajectory("testPath");
		routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
		hi = routine;
		// hicmd = factory.trajectoryCmd("testPath");
		// RobotModeTriggers.autonomous().whileTrue(hicmd).whileTrue(Commands.run(() ->
		// System.out.println("")));
	}

	@Override
	public void autonomousPeriodic()
	{
		System.out.println("running auto...");
		hi.poll();
	}

	public Map<String, Supplier<AutoRoutine>> getAutonomousCommands()
	{
		return Map.of("nothing", () -> factory.newRoutine("nothing"));
	}

	@Override
	public Command getAutonomousCommand()
	{
		return new InstantCommand();
	}

}
