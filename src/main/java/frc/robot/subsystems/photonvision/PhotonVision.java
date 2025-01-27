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

/**
 * A subsystem that interfaces with PhotonVision cameras and pose estimators.
 * This class is responsible for updating the pose estimators with new camera
 * results and providing the latest pose estimates.
 */
public class PhotonVision extends SubsystemBase
{
	private final PhotonCamera[] cameras;
	private final PhotonPoseEstimator[] poseEstimators;
	private final ArrayList<EstimatedRobotPose> poseEstimates;

	/**
	 * Constructs a new PhotonVision subsystem with the given camera names, poses,
	 * and field layout.
	 * 
	 * @param cameraNames The names of the cameras to use.
	 * @param cameraPoses The poses of the cameras relative to the robot.
	 * @param layout      The apriltag field layout to use.
	 */
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

	/**
	 * Returns the latest pose estimates from the PhotonVision cameras.
	 * 
	 * @return The latest pose estimates.
	 */
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

	@AutoLogOutput
	public boolean[] getConnectedStatus()
	{
		boolean[] connected = new boolean[cameras.length];
		for (int i = 0; i < cameras.length; i++)
		{
			connected[i] = cameras[i].isConnected();
		}
		return connected;
	}

	/**
	 * A record that represents a pose estimate from a PhotonVision camera. This
	 * record contains the pose estimate and the timestamp of the estimate.
	 * 
	 * @param pose      The pose estimate.
	 * @param timestamp The timestamp of the estimate.
	 */
	public static record PoseEstimate(Pose2d pose, double timestamp) {
	}
}
