package frc.robot.robots;

import java.util.Map;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.pneumatic.PneumaticHubIORev;
import frc.robot.zippy.ZippyDriverOI;
import frc.robot.zippy.ZippyOI;
import frc.robot.zippy.ZippyProgrammerOI;

public class ZippyContainer implements NFRRobotContainer
{
	private final PneumaticHubIORev m_pneumaticHubIO;

	public ZippyContainer()
	{
		m_pneumaticHubIO = new PneumaticHubIORev(ZippyConstants.pneumaticConstants);
		m_pneumaticHubIO.enable();
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
