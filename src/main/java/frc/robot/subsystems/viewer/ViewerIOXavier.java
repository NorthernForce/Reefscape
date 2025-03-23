package frc.robot.subsystems.viewer;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ViewerIOXavier implements ViewerIO
{
    private final NetworkTable table;
    private final DoubleSubscriber xOffsetSubscriber;
    private final DoubleSubscriber zOffsetSubscriber;
    private final DoubleArraySubscriber postsSubscriber;

    /**
     * Constructs a new ViewerIOXavier.
     */
    public ViewerIOXavier()
    {
        table = NetworkTableInstance.getDefault().getTable("Viewer");
        postsSubscriber = table.getDoubleArrayTopic("Posts").subscribe(new double[]
        {});
        xOffsetSubscriber = table.getDoubleTopic("CandidateMetersX").subscribe(Double.NaN);
        zOffsetSubscriber = table.getDoubleTopic("CandidateMetersZ").subscribe(Double.NaN);
    }

    /**
     * Updates the inputs.
     */
    @Override
    public void updateInputs(ViewerIOInputs inputs)
    {
        inputs.connected = false;
        for (var connection : NetworkTableInstance.getDefault().getConnections())
        {
            if (connection.remote_id.startsWith("skynet"))
            {
                inputs.connected = true;
                break;
            }
        }
        double xOffset = xOffsetSubscriber.get();
        double zOffset = zOffsetSubscriber.get();
        inputs.postDetected = xOffset != Float.NaN && zOffset != Float.NaN;
        inputs.postXOffset = xOffset;
        inputs.postZOffset = zOffset;
    }
}
