package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.*;

public interface WristIO
{
    @AutoLog
    public static class WristIOInputs
    {
        public Angle encoderAngle;
        public Current motorCurrent;
        public Temperature motorTemperature;
        public boolean motorPresent;
    }

    public default void updateInputs(WristIOInputs inputs)
    {
    };

    public default void set(double speed)
    {
    };

    public default Command getMoveToAngleCommand(Angle angle)
    {
        return Commands.runOnce(() ->
        {
        });
    };

    public default Angle getAngle()
    {
        return Degrees.of(0);
    };

    public default Angle getTargetAngle()
    {
        return Degrees.of(0);
    };
}