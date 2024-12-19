package frc.robot.subsystems;

import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for note detection running on Xavier coprocessor
 */
public class Xavier extends SubsystemBase
{
	protected final NetworkTableInstance instance;
	protected final NetworkTable table;
	protected final FloatSubscriber xRadSubscriber;

	/**
	 * Create a new Xavier subsystem
	 * 
	 * @param config the configuration for this subsystem
	 */
	public Xavier(String tableName)
	{
		instance = NetworkTableInstance.getDefault();
		table = instance.getTable(tableName);
		xRadSubscriber = table.getFloatTopic("note_rad").subscribe(Float.NaN);
	}

	/**
	 * Gets the radian of the note the Xavier is most confident about
	 * 
	 * @return x radian of note relative to center of robot (NaN if none detected)
	 */
	public float getYawRadians()
	{
		return xRadSubscriber.get();
	}

	/**
	 * Gets whether the Xavier/Nano is connected or not
	 * 
	 * @return whether Xavier/Nano is connected or not
	 */
	public boolean isConnected()
	{
		for (var connection : instance.getConnections())
		{
			if (connection.remote_id.contains("skynet"))
			{
				return true;
			}
		}
		return false;
	}
}