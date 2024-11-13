package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraManager extends SubsystemBase
{

	private final CameraManagerIO io;
	private final CameraManagerIOInputsAutoLogged inputs = new CameraManagerIOInputsAutoLogged();

	/**
	 * Class to manage any number PhotonVisionCamera objects
	 * 
	 * @param photonVisionCamera The PhotonVisionCamera objects to be managed
	 */
	public CameraManager(CameraManagerIO io)
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
		io.updateInputs(inputs);
		Logger.processInputs(getName(), inputs);
	}
}
