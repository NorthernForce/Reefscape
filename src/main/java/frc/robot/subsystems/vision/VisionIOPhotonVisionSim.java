package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision
{
    private final Supplier<Pose2d> poseSupplier;
    protected static VisionSystemSim visionSim;

	public VisionIOPhotonVisionSim(Supplier<Pose2d> poseSupplier, PhotonVisionCameraSim... photonVisionCameras)
    {
        this.poseSupplier = poseSupplier;

        if (visionSim == null)
        {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo));
        }

        for (PhotonVisionCameraSim cam : photonVisionCameras)
        {
            visionSim.addCamera(cam.getCameraSim(), cam.getRobotToCamera());
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, int camIndex)
    {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs, camIndex);
    }
}
