package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase
{

	private final VisionIO io;
	private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

	/**
	 * Class to manage any number PhotonVisionCamera objects
	 * 
	 * @param photonVisionCamera The PhotonVisionCamera objects to be managed
	 */
	public Vision(VisionIO io)
	{
		this.io = io;
	}

	/**
	 * Update inputs without running the rest of the periodic logic. This is useful
	 * since these updates need to be properly thread-locked.
	 */
	@Override
	public void periodic()
	{
        for (int i = 0; i < io.getNumberOfCameras(); i++)
        {
		    io.updateInputs(inputs, i);
		    Logger.processInputs(getName(), inputs);
        }
	}
}
