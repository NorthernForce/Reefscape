package frc.robot.zippy;

import java.util.Map;
import java.util.function.Supplier;

import javax.xml.stream.FactoryConfigurationError;

import org.northernforce.util.NFRRobotContainer;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

	public ZippyContainer()
	{
		drive = new PhoenixCommandDrive(ZippyTunerConstants.DrivetrainConstants,
				ZippyConstants.DrivetrainConstants.MAX_SPEED, ZippyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
				ZippyTunerConstants.FrontLeft, ZippyTunerConstants.FrontRight, ZippyTunerConstants.BackLeft,
				ZippyTunerConstants.BackRight);
		drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(alliance));
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
			drive.runVelocity(speeds);
		}, true, drive);
		factory.bind("test", Commands.run(() -> System.out.println("test binding ran!"))).bind("test2",
				Commands.run(() -> System.out.println("yup yup!")));
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
