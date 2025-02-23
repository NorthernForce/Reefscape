package frc.robot.subsystems.viewer;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ViewerIOXavier implements ViewerIO
{
    private final NetworkTable table;
    private final BooleanSubscriber hasPostInImageSubscriber;
    private final DoubleSubscriber postOffsetSubscriber;
    private final DoubleSubscriber postDistanceSubscriber;
    private final DoubleSubscriber centerDistanceSubscriber;

    /**
     * Constructs a new ViewerIOXavier.
     */
    public ViewerIOXavier()
    {
        table = NetworkTableInstance.getDefault().getTable("Xavier");
        hasPostInImageSubscriber = table.getBooleanTopic("HasPost").subscribe(false);
        postOffsetSubscriber = table.getDoubleTopic("PostOffset").subscribe(0.0);
        postDistanceSubscriber = table.getDoubleTopic("PostDistance").subscribe(0.0);
        centerDistanceSubscriber = table.getDoubleTopic("CenterDistance").subscribe(0.0);
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
            if (connection.remote_id.startsWith("Xavier"))
            {
                inputs.connected = true;
                break;
            }
        }
        inputs.hasPostInImage = hasPostInImageSubscriber.get();
        inputs.postOffset = Meters.of(postOffsetSubscriber.get());
        inputs.postDistance = Meters.of(postDistanceSubscriber.get());
        inputs.centerDistance = Meters.of(centerDistanceSubscriber.get());
    }
}
