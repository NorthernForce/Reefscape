package frc.robot.subsystems.specialstick;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.specialstick.sensor.AlgaeRemoverSensorIOInputsAutoLogged;
import frc.robot.subsystems.specialstick.sensor.AlgaeRemoverSensorIO;

public class SpecialStick extends SubsystemBase
{
    private final SpecialStickIO io;
    private final AlgaeRemoverSensorIO sensorIO;
    private final AlgaeRemoverSensorIOInputsAutoLogged sensorInputs = new AlgaeRemoverSensorIOInputsAutoLogged();
    private final SpecialStickIOInputsAutoLogged inputs = new SpecialStickIOInputsAutoLogged();
    private final double returningSpeed;
    private final double removingSpeed;

    public SpecialStick(SpecialStickIO algaeRemoverIO, AlgaeRemoverSensorIO algaeSensorIO, double removingSpeed,
            double returningSpeed)
    {
        io = algaeRemoverIO;
        Logger.processInputs(getName(), inputs);
        sensorIO = algaeSensorIO;
        Logger.processInputs(getName(), sensorInputs);
        this.removingSpeed = removingSpeed;
        this.returningSpeed = returningSpeed;
    }

    @AutoLogOutput
    public boolean hasReachedTop()
    {
        return sensorInputs.reachedTop;
    }

    public void stop()
    {
        io.stopMotor();
    }

    @Override
    public void periodic()
    {
        sensorIO.updateInputs(sensorInputs);
        Logger.processInputs(getName() + "Sensor", sensorInputs);
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    public class RemoveAlgaeCommand extends Command
    {
        public RemoveAlgaeCommand()
        {
            addRequirements(SpecialStick.this);
        }

        @Override
        public void execute()
        {
            System.out.println("Here");
            io.set(removingSpeed);
        }

        @Override
        public void end(boolean interrupted)
        {
        }
    }

    public class ReturnArmCommand extends Command
    {
        public ReturnArmCommand()
        {
            addRequirements(SpecialStick.this);
        }

        @Override
        public void execute()
        {
            if (!hasReachedTop())
            {
                io.set(-returningSpeed);
            } else
            {
                io.set(0);
            }
        }

        @Override
        public boolean isFinished()
        {
            return hasReachedTop();
        }

        @Override
        public void end(boolean interrupted)
        {
        }
    }

    public Command deploySpecialStick()
    {
        return new RemoveAlgaeCommand();
    }

    public Command pullOutSpecialStick()
    {
        return new ReturnArmCommand();
    }
}
