package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.math.geometry.Rotation2d;

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

    public default void run(double speed)
    {
    };

    public default void runToAngle(Rotation2d angle)
    {
    };
}