package frc.robot.subsystems.viewer;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ViewerIOXavier implements ViewerIO
{
    private final NetworkTable table;
    private final DoubleArraySubscriber postOffsetSubscriber;
    private final DoubleArraySubscriber postDistanceSubscriber;

    /**
     * Constructs a new ViewerIOXavier.
     */
    public ViewerIOXavier()
    {
        table = NetworkTableInstance.getDefault().getTable("Viewer");
        postOffsetSubscriber = table.getDoubleArrayTopic("PostOffsets").subscribe(new double[]
        {});
        postDistanceSubscriber = table.getDoubleArrayTopic("PostDistances").subscribe(new double[]
        {});
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
        inputs.postOffsetMeters = postOffsetSubscriber.get();
        inputs.postDistanceMeters = postDistanceSubscriber.get();
    }
}
