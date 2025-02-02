package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.units.measure.*;

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

    public void set(double speed)
    {
        io.set(speed);
    }

    public Command getMoveToAngleCommand(Angle angle)
    {
        return Commands.runOnce(() -> io.moveToAngle(angle));
    }

    public Command getStopCommand()
    {
        return Commands.runOnce(() -> io.set(0));
    }

    public Angle getAngle()
    {
        return inputs.encoderAngle;
    }

    public Angle getTargetAngle()
    {
        return io.getTargetAngle();
    }

    public boolean isAtTargetPosition()
    {
        return Math.abs(getAngle().in(Degrees) - getTargetAngle().in(Degrees)) < errorTolerance;
    }

    public boolean isAtPosition(Angle angle)
    {
        return Math.abs(getAngle().in(Degrees) - angle.in(Degrees)) < errorTolerance;
    }

    public void resetEncoderAngle(Angle angle)
    {
        io.resetEncoderAngle(angle);
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(getName() + "/Encoder", inputs);
    }
}
