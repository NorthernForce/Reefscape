package frc.robot.subsystems.vision;

public class VisionIOPhotonVision implements VisionIO
{
	private final PhotonVisionCamera[] photonVisionCameras;
	protected boolean[] connected;
	protected int numberOfCameras;

	/**
	 * Class to manage any number PhotonVisionCamera objects
	 * 
	 * @param photonVisionCamera The PhotonVisionCamera objects to be managed
	 */
	public VisionIOPhotonVision(PhotonVisionCamera... photonVisionCameras)
	{
		numberOfCameras = photonVisionCameras.length;
		connected = new boolean[numberOfCameras];
		this.photonVisionCameras = photonVisionCameras;
	}

	@Override
	public void updateInputs(VisionIOInputs inputs, int camIndex)
	{
		photonVisionCameras[camIndex].updateInputs(inputs, camIndex);
	}

	public boolean[] getConnectedCameras()
	{
		boolean[] connectedCameras = new boolean[numberOfCameras];
		for (int i = 0; i < connected.length; i++)
		{
			PhotonVisionCamera cam = photonVisionCameras[i];
			connectedCameras[i] = cam.isConnected();
		}
		connected = connectedCameras;
		return connected;
	}

    public int getNumberOfCameras()
    {
        return numberOfCameras;
    }
}
