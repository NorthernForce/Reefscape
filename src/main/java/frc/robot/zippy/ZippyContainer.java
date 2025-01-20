package frc.robot.zippy;

import java.util.function.Supplier;

import org.northernforce.util.NFRRobotContainer;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.subsystems.photonvision.PhotonVision;
import frc.robot.zippy.constants.ZippyConstants;
import frc.robot.zippy.constants.ZippyTunerConstants;
import frc.robot.zippy.oi.ZippyDriverOI;
import frc.robot.zippy.oi.ZippyOI;
import frc.robot.zippy.oi.ZippyProgrammerOI;

public class ZippyContainer implements NFRRobotContainer
{
	private final PhoenixCommandDrive drive;
	private final Supplier<Alliance> allianceSupplier = () -> DriverStation.getAlliance().orElse(Alliance.Red);
	private Alliance alliance = allianceSupplier.get();
	private final PhotonVision vision;
	private final PowerDistribution pdh;

	public ZippyContainer()
	{
		drive = new PhoenixCommandDrive(ZippyTunerConstants.DrivetrainConstants,
				ZippyConstants.DrivetrainConstants.MAX_SPEED, ZippyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
				ZippyTunerConstants.FrontLeft, ZippyTunerConstants.FrontRight, ZippyTunerConstants.BackLeft,
				ZippyTunerConstants.BackRight);
		drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(alliance));
		vision = new PhotonVision(ZippyConstants.visionConstants.cameraNames(),
				ZippyConstants.visionConstants.cameraTransforms(), ZippyConstants.visionConstants.aprilTagLayout());
		pdh = new PowerDistribution(40, ModuleType.kRev);
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
		if (alliance != allianceSupplier.get())
		{
			alliance = allianceSupplier.get();
			drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(allianceSupplier.get()));
		}
		for (var poseEstimate : vision.getPoseEstimates())
		{
			drive.addVisionMeasurement(poseEstimate.pose(), Utils.fpgaToCurrentTime(poseEstimate.timestamp()));
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
