package frc.robot.subsystems.viewer;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
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

    public static record ViewerTarget(Distance xDistance, Distance zDistance) {

    }

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

    @AutoLogOutput
    public boolean isPresent()
    {
        return inputs.connected;
    }

    public Optional<ViewerTarget> getTarget()
    {
        if (inputs.connected && inputs.postDetected)
        {
            return Optional.of(new ViewerTarget(Distance.ofBaseUnits(inputs.postXOffset, Meters),
                    Distance.ofBaseUnits(inputs.postZOffset, Meters)));
        }
        return Optional.empty();
    }
}
