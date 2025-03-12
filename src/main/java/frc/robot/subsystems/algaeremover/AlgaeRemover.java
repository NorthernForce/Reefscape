package frc.robot.subsystems.algaeremover;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeremover.sensor.AlgaeRemoverSensorIO;
import frc.robot.subsystems.algaeremover.sensor.AlgaeRemoverSensorIOInputsAutoLogged;

public class AlgaeRemover extends SubsystemBase
{
    private final AlgaeRemoverIO io;
    private final AlgaeRemoverSensorIO sensorIO;
    private final AlgaeRemoverSensorIOInputsAutoLogged sensorInputs = new AlgaeRemoverSensorIOInputsAutoLogged();
    private final AlgaeRemoverIOInputsAutoLogged inputs = new AlgaeRemoverIOInputsAutoLogged();
    private final double returningSpeed;
    private final double removingSpeed;

    public AlgaeRemover(AlgaeRemoverIO algaeRemoverIO, AlgaeRemoverSensorIO algaeSensorIO, double removingSpeed,
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
            addRequirements(AlgaeRemover.this);
        }

        @Override
        public void initialize()
        {
            io.set(removingSpeed);
        }

        @Override
        public void end(boolean interrupted)
        {
            stop();
        }
    }

    public class ReturnArmCommand extends Command
    {
        public ReturnArmCommand()
        {
            addRequirements(AlgaeRemover.this);
        }

        @Override
        public void execute()
        {
            if (!hasReachedTop())
            {
                io.set(-returningSpeed);
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
            stop();
        }
    }

    public Command removeAlgae()
    {
        return new RemoveAlgaeCommand();
    }

    public Command returnArm()
    {
        return new ReturnArmCommand();
    }
}
