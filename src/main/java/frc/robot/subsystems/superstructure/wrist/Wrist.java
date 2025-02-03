package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.units.measure.*;

/**
 * Class to control the robot's wrist joint
 */
public class Wrist extends SubsystemBase
{
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private final double errorTolerance;

    public Wrist(WristIO io, double errorToleranceDegrees)
    {
        this.io = io;
        errorTolerance = errorToleranceDegrees;
    }

    /**
     * Sets the wrist to run at the desired speed
     * 
     * @param speed (0.0 - 1.0) The speed to run the motor at
     */
    public void set(double speed)
    {
        io.set(speed);
    }

    /**
     * Gets the command to move the wrist to a certain angle
     * 
     * @param angle The angle to move to
     * @return The move to angle command
     */
    public Command getMoveToAngleCommand(Angle angle)
    {
        return Commands.runOnce(() -> io.moveToAngle(angle));
    }

    /**
     * Gets the command to stop moving the wrist
     * 
     * @return The stop moving command
     */
    public Command getStopCommand()
    {
        return Commands.runOnce(() -> io.set(0));
    }

    /**
     * Gets the current angle of the wrist joint
     * 
     * @return Angle of the wrist
     */
    public Angle getAngle()
    {
        return inputs.encoderAngle;
    }

    /**
     * Gets the current target angle of the wrist
     * 
     * @return Target angle of the wrist
     */
    public Angle getTargetAngle()
    {
        return inputs.targetAngle;
    }

    /**
     * Gets whether or not the difference between the current angle and the target
     * angle of the wrist is below the error tolerance
     * 
     * @return true if the current wrist angle is close enough to the target angle,
     *         false if not
     */
    public boolean isAtTargetPosition()
    {
        return Math.abs(getAngle().in(Degrees) - getTargetAngle().in(Degrees)) < errorTolerance;
    }

    /**
     * Gets whether or not the difference between the current angle and the inputted
     * angle of the wrist is below the error tolerance
     * 
     * @param angle The angle to compare
     * @return true if the current wrist angle is close enough to the inputted
     *         angle, false if not
     */
    public boolean isAtPosition(Angle angle)
    {
        return Math.abs(getAngle().in(Degrees) - angle.in(Degrees)) < errorTolerance;
    }

    /**
     * Sets the current angle of the motor's cancoder to the inputted angle
     * 
     * @param angle The angle to set to
     */
    public Command resetEncoderAngle(Angle angle)
    {
        return Commands.runOnce(() -> io.resetEncoderAngle(angle));
    }

    /**
     * Update and log the wrist's inputs
     */
    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(getName() + "/Wrist", inputs);
    }
}
