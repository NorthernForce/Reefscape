package frc.robot.subsystems.viewer;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ViewerIOXavier implements ViewerIO
{
    private final NetworkTable table;
    private final DoubleSubscriber xOffsetSubscriber;
    private int i = 0;

    /**
     * Constructs a new ViewerIOXavier.
     */
    public ViewerIOXavier()
    {
        table = NetworkTableInstance.getDefault().getTable("Viewer");
        xOffsetSubscriber = table.getDoubleTopic("CandidateMetersX").subscribe(Double.NaN);
    }

    /**
     * Updates the inputs.
     */
    @Override
    public void updateInputs(ViewerIOInputs inputs)
    {
        if (i % 200 == 0)
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
        }
        i++;
        double xOffset = -xOffsetSubscriber.get();
        inputs.postDetected = !Double.isNaN(xOffset);
        inputs.postXOffset = xOffset;
    }
}
