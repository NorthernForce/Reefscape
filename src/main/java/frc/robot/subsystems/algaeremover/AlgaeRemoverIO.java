package frc.robot.subsystems.algaeremover;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface AlgaeRemoverIO
{
    @AutoLog
    public static class AlgaeRemoverIOInputs
    {
        public boolean isPresent = false;
        public Current current = Amps.of(0);
        public Voltage voltage = Volts.of(0);
        public Angle position = Rotations.of(0);
        public AngularVelocity velocity = RotationsPerSecond.of(0);
        public Temperature temperature = Celsius.of(0);
    }

    public default void set(double speed)
    {
    }

    public default void stopMotor()
    {
    }

    public default void updateInputs(AlgaeRemoverIOInputs inputs)
    {
    }
}
