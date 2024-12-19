package frc.robot.robots;

import java.util.Map;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.zippy.ZippyDriverOI;
import frc.robot.zippy.ZippyOI;
import frc.robot.zippy.ZippyProgrammerOI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIO;

public class ZippyContainer implements NFRRobotContainer
{
	private final Vision vision;
	private final Drive drive = new Drive(null, 0, 0, 0, 0, 0, null, new ModuleIO[0]);

	public ZippyContainer()
	{
		switch (Constants.kCurrentMode)
		{
		case REAL:
			vision = new Vision(drive::addVisionMeasurement, ZippyConstants.visionConstants,
					new VisionIOPhotonVision(ZippyConstants.visionConstants.cameraNames()[0],
							ZippyConstants.visionConstants.cameraTransforms()[0]),
					new VisionIOPhotonVision(ZippyConstants.visionConstants.cameraNames()[1],
							ZippyConstants.visionConstants.cameraTransforms()[0]),
					new VisionIOPhotonVision(ZippyConstants.visionConstants.cameraNames()[2],
							ZippyConstants.visionConstants.cameraTransforms()[0]),
					new VisionIOPhotonVision(ZippyConstants.visionConstants.cameraNames()[3],
							ZippyConstants.visionConstants.cameraTransforms()[0]));
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
	public Map<String, Command> getAutonomousOptions()
	{
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getAutonomousOptions'");
	}

	@Override
	public Map<String, Pose2d> getStartingLocations()
	{
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getStartingLocations'");
	}

	@Override
	public Pair<String, Command> getDefaultAutonomous()
	{
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getDefaultAutonomous'");
	}

	@Override
	public void setInitialPose(Pose2d pose)
	{
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'setInitialPose'");
	}

}
