package frc.robot.subsystems.photonvision;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** If only we had two RoboRIO 2's */
public class PhotonVision extends SubsystemBase
{
	private final PhotonCamera[] cameras;
	private final PhotonPoseEstimator[] poseEstimators;
	private final ArrayList<EstimatedRobotPose> poseEstimates;

	public PhotonVision(String[] cameraNames, Transform3d[] cameraPoses, AprilTagFieldLayout layout)
	{
		cameras = new PhotonCamera[cameraNames.length];
		poseEstimators = new PhotonPoseEstimator[cameraNames.length];
		for (int i = 0; i < cameraNames.length; i++)
		{
			cameras[i] = new PhotonCamera(cameraNames[i]);
			poseEstimators[i] = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
					cameraPoses[i]);
		}
		poseEstimates = new ArrayList<>();
	}

	@Override
	public void periodic()
	{
		poseEstimates.clear();
		for (int i = 0; i < cameras.length; i++)
		{
			for (var result : cameras[i].getAllUnreadResults())
			{
				var opt = poseEstimators[i].update(result);
				if (opt.isPresent())
				{
					poseEstimates.add(opt.get());
				}
			}
		}
	}

	@AutoLogOutput
	public PoseEstimate[] getPoseEstimates()
	{
		PoseEstimate[] poses = new PoseEstimate[poseEstimates.size()];
		for (int i = 0; i < poses.length; i++)
		{
			poses[i] = new PoseEstimate(poseEstimates.get(i).estimatedPose.toPose2d(),
					poseEstimates.get(i).timestampSeconds);
		}
		return poses;
	}

	public static record PoseEstimate(Pose2d pose, double timestamp) {
	}
}
