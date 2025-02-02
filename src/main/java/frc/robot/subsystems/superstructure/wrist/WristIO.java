package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
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

    public default void moveToAngle(Angle angle)
    {
    };

    public default Angle getTargetAngle()
    {
        return Degrees.of(0);
    };

    public default void resetEncoderAngle(Angle angle)
    {
    };
}