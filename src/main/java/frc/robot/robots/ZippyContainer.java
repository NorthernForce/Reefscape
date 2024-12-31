package frc.robot.robots;

import java.util.Map;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.zippy.ZippyDriverOI;
import frc.robot.zippy.ZippyOI;
import frc.robot.zippy.ZippyProgrammerOI;

public class ZippyContainer implements NFRRobotContainer
{
	public final Drive drive = new Drive(null, 0, 0, 0, 0, 0, null, null);

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

		if (DriverStation.getJoystickIsXbox(0))
		{
			zippyOI.bindDriverToXboxController(this, new CommandXboxController(0));
		} else
		{
			zippyOI.bindDriverToJoystick(this, new CommandGenericHID(0));
		}

		if (DriverStation.getJoystickIsXbox(1))
		{
			zippyOI.bindManipulatorToXboxController(this, new CommandXboxController(1));
		} else
		{
			zippyOI.bindManipulatorToJoystick(this, new CommandGenericHID(1));
		}
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
