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

    public static record ViewerTarget(Distance xDistance, Distance yDistance) {
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

    @AutoLogOutput
    public ViewerTarget[] getTargets()
    {
        if (inputs.connected)
        {
            double[] postDistances = inputs.postDistanceMeters;
            double[] postOffsets = inputs.postOffsetMeters;
            if (postDistances.length == postOffsets.length)
            {
                ViewerTarget[] targets = new ViewerTarget[postDistances.length];
                for (int i = 0; i < postDistances.length; i++)
                {
                    targets[i] = new ViewerTarget(Meters.of(postDistances[i]), Meters.of(postOffsets[i]));
                }
                return targets;
            }
        }
        return new ViewerTarget[0];
    }

    public Optional<ViewerTarget> getBestTarget()
    {
        var targets = getTargets();
        if (inputs.connected && targets.length > 0)
        {
            ViewerTarget bestTarget = targets[0];
            for (var target : targets)
            {
                if (target.yDistance().lte(bestTarget.yDistance()))
                {
                    bestTarget = target;
                }
            }
            if (bestTarget.xDistance().lte(Meters.of(0.5)))
            {
                return Optional.of(bestTarget);
            }
        }
        return Optional.empty();
    }
}
