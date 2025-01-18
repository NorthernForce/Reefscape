package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision
{
	private static VisionSystemSim visionSim;

	private final Supplier<Pose2d> poseSupplier;
	private final PhotonCameraSim cameraSim;

	public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier)
	{
		super(name, robotToCamera);
		this.poseSupplier = poseSupplier;

		if (visionSim == null)
		{
			visionSim = new VisionSystemSim("main");
			visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape));
		}

		var cameraPropertes = new SimCameraProperties();
		cameraSim = new PhotonCameraSim(camera, cameraPropertes);
		visionSim.addCamera(cameraSim, robotToCamera);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs)
	{
		visionSim.update(poseSupplier.get());
		super.updateInputs(inputs);
	}

	@Override
	public String getCameraName()
	{
		return camera.getName();
	}
}
