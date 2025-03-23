package frc.robot.subsystems.viewer;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ViewerIOXavier implements ViewerIO
{
    private final NetworkTable table;
    private final DoubleArraySubscriber postsSubscriber;

    /**
     * Constructs a new ViewerIOXavier.
     */
    public ViewerIOXavier()
    {
        table = NetworkTableInstance.getDefault().getTable("Viewer");
        postsSubscriber = table.getDoubleArrayTopic("Posts").subscribe(new double[]
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
        var posts = postsSubscriber.get();
        inputs.postDistanceMeters = new double[posts.length / 2];
        inputs.postOffsetMeters = new double[posts.length / 2];
        for (int i = 0; i < posts.length / 2; i++)
        {
            inputs.postDistanceMeters[i] = posts[i * 2];
            inputs.postOffsetMeters[i] = posts[i * 2 + 1];
        }
    }
}
