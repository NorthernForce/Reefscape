package frc.robot.zippy;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.zippy.constants.ZippyConstants;
import frc.robot.zippy.constants.ZippyTunerConstants;
import frc.robot.zippy.oi.ZippyDriverOI;
import frc.robot.zippy.oi.ZippyOI;
import frc.robot.zippy.oi.ZippyProgrammerOI;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

public class ZippyContainer implements NFRRobotContainer
{
	private final PhoenixCommandDrive drive;
	private final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
	private final Vision vision;

	public ZippyContainer()
	{
		drive = new PhoenixCommandDrive(ZippyTunerConstants.DrivetrainConstants,
				ZippyConstants.DrivetrainConstants.MAX_SPEED, ZippyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
				ZippyTunerConstants.FrontLeft, ZippyTunerConstants.FrontRight, ZippyTunerConstants.BackLeft,
				ZippyTunerConstants.BackRight);
		drive.setOperatorPerspectiveForward(FieldConstants.getFieldRotation(alliance));

		switch (Constants.kCurrentMode)
		{
		case REAL:
			vision = new Vision(drive::addVisionMeasurement, ZippyConstants.visionConstants,
					new VisionIOPhotonVision(ZippyConstants.visionConstants.cameraNames()[0],
							ZippyConstants.visionConstants.cameraTransforms()[0]),
					new VisionIOPhotonVision(ZippyConstants.visionConstants.cameraNames()[1],
							ZippyConstants.visionConstants.cameraTransforms()[1]),
					new VisionIOPhotonVision(ZippyConstants.visionConstants.cameraNames()[2],
							ZippyConstants.visionConstants.cameraTransforms()[2]),
					new VisionIOPhotonVision(ZippyConstants.visionConstants.cameraNames()[3],
							ZippyConstants.visionConstants.cameraTransforms()[3]));
			break;
		case SIM:
			vision = new Vision(drive::addVisionMeasurement, ZippyConstants.visionConstants,
					new VisionIOPhotonVisionSim(ZippyConstants.visionConstants.cameraNames()[0],
							ZippyConstants.visionConstants.cameraTransforms()[0], drive::getPose),
					new VisionIOPhotonVisionSim(ZippyConstants.visionConstants.cameraNames()[1],
							ZippyConstants.visionConstants.cameraTransforms()[1], drive::getPose),
					new VisionIOPhotonVisionSim(ZippyConstants.visionConstants.cameraNames()[2],
							ZippyConstants.visionConstants.cameraTransforms()[2], drive::getPose),
					new VisionIOPhotonVisionSim(ZippyConstants.visionConstants.cameraNames()[3],
							ZippyConstants.visionConstants.cameraTransforms()[3], drive::getPose));
			break;
		default:
			vision = new Vision(drive::addVisionMeasurement, ZippyConstants.visionConstants, new VisionIO()
			{
			}, new VisionIO()
			{
			}, new VisionIO()
			{
			}, new VisionIO()
			{
			});
			break;
		}
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
		drive.resetPose(new Pose2d());
	}

	@Override
	public Command getAutonomousCommand()
	{
		return new InstantCommand();
	}

}
