package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class PhotonVisionCameraSim extends PhotonVisionCamera
{
    private final PhotonCameraSim cameraSim;
    private final Transform3d robotToCamera;

	/**
	 * Wrapper class for a PhotonCamera and PhotonPoseEstimator
	 * 
	 * @param aprilTagFieldLayout The layout of the apriltags on the field
	 * @param robotToCam          Transform between the robot origin and the camera
	 * @param cameraNum           Used to give each camera a unique name
	 */
	public PhotonVisionCameraSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier)
	{
        super(name, robotToCamera);
        this.robotToCamera = robotToCamera;
        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(photonCamera, cameraProperties);
        VisionIOPhotonVisionSim.visionSim.addCamera(cameraSim, robotToCamera);
	}

    @Override
    public void updateInputs(VisionIOInputs inputs, int camIndex)
    {
        super.updateInputs(inputs, camIndex);
    }

    public PhotonCameraSim getCameraSim()
    {
        return cameraSim;
    }

    public Transform3d getRobotToCamera()
    {
        return robotToCamera;
    }
}
