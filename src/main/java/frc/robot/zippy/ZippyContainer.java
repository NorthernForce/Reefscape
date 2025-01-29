package frc.robot.zippy;

import java.util.function.Supplier;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.subsystems.reefscape.ReefDisplayIOSwing;
import frc.robot.zippy.constants.ZippyConstants;
import frc.robot.zippy.constants.ZippyTunerConstants;
import frc.robot.zippy.dashboard.ZippyDashboard;
import frc.robot.zippy.dashboard.ZippyDashboardIOElastic;
import frc.robot.zippy.dashboard.ZippyDashboardIOIMGUI;
import frc.robot.zippy.oi.ZippyDriverOI;
import frc.robot.zippy.oi.ZippyOI;
import frc.robot.zippy.oi.ZippyProgrammerOI;

public class ZippyContainer implements NFRRobotContainer
{
	private final PhoenixCommandDrive drive;
	private final Supplier<Alliance> allianceSupplier = () -> DriverStation.getAlliance().orElse(Alliance.Red);
	private Alliance alliance = allianceSupplier.get();
	private final ZippyDashboard dashboard;

	public ZippyContainer()
	{
		dashboard = new ZippyDashboard(new ReefDisplayIOSwing("ReefDisplay"), new ZippyDashboardIOElastic(),
				new ZippyDashboardIOIMGUI());
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

	public ZippyDashboard getDashboard()
	{
		return dashboard;
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
		if (alliance != allianceSupplier.get())
		{
			alliance = allianceSupplier.get();
			drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(allianceSupplier.get()));
		}
	}

	@Override
	public void autonomousInit()
	{
		drive.resetPose(new Pose2d());
	}

	@Override
	public Command getAutonomousCommand()
	{
		return new InstantCommand();
	}

}
