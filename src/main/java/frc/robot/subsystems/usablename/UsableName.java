package frc.robot.subsystems.usablename;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Xavier;

public class UsableName extends SubsystemBase implements UsableNameIO
{
	private final Xavier xavier;
	public UsableNameIODataAutoLogged data;

	public UsableName(Xavier xavier)
	{
		this.xavier = xavier;
		this.data = new UsableNameIODataAutoLogged();

	}

	public void updateInputs()
	{
		float yaw = xavier.getYawRadians();
		data.connected = xavier.isConnected();
		data.detected = yaw == Float.NaN;
		data.yawRadians = yaw;
	}

	public UsableNameIODataAutoLogged getInputs()
	{
		return this.data;
	}
}
