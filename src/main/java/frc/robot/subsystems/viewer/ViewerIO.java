package frc.robot.subsystems.viewer;

import org.littletonrobotics.junction.AutoLog;

/**
 * base class of what ViewerIO interface should be like, (will most certainly be
 * modified in the future as this is purely an example)
 */
public interface ViewerIO
{
	/**
	 * record containing the data from coprocessor
	 */
	@AutoLog
	public class ViewerIOData
	{
		float yawRadians = Float.NaN;
		boolean detected = false;
		boolean connected = false;
	}

	/**
	 * example function of ViewerIO
	 * 
	 * @return data from coprocessor
	 */
	public void updateInputs();

	public ViewerIODataAutoLogged getInputs();
}
