package frc.robot.subsystems.viewer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Viewer extends SubsystemBase
{
    private final ViewerIO io;
    private final ViewerIOInputsAutoLogged inputs;
    private final Alert viewerMissingAlert;

    public Viewer(ViewerIO io)
    {
        this.io = io;
        inputs = new ViewerIOInputsAutoLogged();
        viewerMissingAlert = new Alert("Viewer is missing", AlertType.kWarning);
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
        viewerMissingAlert.set(!inputs.connected);
    }

    public Distance getCenterDistance()
    {
        return inputs.centerDistance;
    }

    public Distance getPostDistance()
    {
        return inputs.postDistance;
    }

    public Distance getPostOffset()
    {
        return inputs.postOffset;
    }

    public boolean isPresent()
    {
        return inputs.connected;
    }
}
