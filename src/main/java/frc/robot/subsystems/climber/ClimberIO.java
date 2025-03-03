package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO for the climber.
 */

public interface ClimberIO
{
    /**
     * ClimberIOInputs class.
     */

    @AutoLog
    public static class ClimberIOInputs
    {
        public Angle position = Degrees.of(0);
        public boolean present = false;
        public Temperature temperature = Celsius.of(0);
        public Current current = Amps.of(0);
    }

    /**
     * run method for the ClimberIO class at a certain speed.
     * 
     * @param speed
     */

    public default void run(double speed)
    {
    }

    /**
     * stop method for the ClimberIO class.
     */

    public default void stop()
    {
    }

    /**
     * update inputs method for the ClimberIO class.
     * 
     * @param inputs inputs for the climber
     */

    public default void updateInputs(ClimberIOInputs inputs)
    {
    }

    public default void runTo(Angle position)
    {
    }
}