package frc.robot.subsystems.viewer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Xavier;

public class Viewer extends SubsystemBase implements ViewerIO
{
	private final Xavier xavier;
	public ViewerIODataAutoLogged data;

	public Viewer(Xavier xavier)
	{
		this.xavier = xavier;
		this.data = new ViewerIODataAutoLogged();

	}

	public void updateInputs()
	{
		float yaw = xavier.getYawRadians();
		data.connected = xavier.isConnected();
		data.detected = yaw == Float.NaN;
		data.yawRadians = yaw;
	}

	public ViewerIODataAutoLogged getInputs()
	{
		return this.data;
	}
}
