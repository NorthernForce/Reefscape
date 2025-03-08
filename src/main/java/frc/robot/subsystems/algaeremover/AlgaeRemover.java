package frc.robot.subsystems.algaeremover;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeremover.sensor.AlgaeRemoverSensorIO;
import frc.robot.subsystems.algaeremover.sensor.AlgaeRemoverSensorIOInputsAutoLogged;
import frc.robot.subsystems.algaeremover.sensor.AlgaeRemoverSensorIO.AlgaeRemoverSensorIOInputs;

public class AlgaeRemover extends SubsystemBase
{
    private AlgaeRemoverIO io;
    private AlgaeRemoverSensorIO sensorIO;
    private AlgaeRemoverSensorIOInputsAutoLogged sensorInputs = new AlgaeRemoverSensorIOInputsAutoLogged();
    private AlgaeRemoverIOInputsAutoLogged inputs = new AlgaeRemoverIOInputsAutoLogged();
    private double returningSpeed = 0.5;
    private double removingSpeed = 0.5;

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

    public void removeAlgae()
    {
        io.removeAlgae(removingSpeed);
    }

    public void returnArm()
    {
        io.returnArm(returningSpeed);
    }

    @AutoLogOutput
    public boolean hasReachedTop()
    {
        return sensorInputs.reachedTop;
    }

    public void algaeRemoved(boolean algaeRemoved)
    {
        io.algaeRemoved(algaeRemoved);
    }

    public boolean getAlgaeRemoved()
    {
        return io.getAlgaeRemoved();
    }

    public void stopMotor()
    {
        io.stopMotor();
    }

    @Override
    public void periodic()
    {
        sensorIO.updateInputs(sensorInputs);
        io.updateInputs(inputs);
    }

    public class removeAlgaeCommand extends Command
    {
        public removeAlgaeCommand()
        {
            addRequirements(AlgaeRemover.this);
        }

        @Override
        public void initialize()
        {
            io.removeAlgae(removingSpeed);
        }

        @Override
        public void execute()
        {
            io.removeAlgae(removingSpeed);
        }

        @Override
        public void end(boolean interrupted)
        {
            io.stopMotor();
        }
    }

    public class returnArmCommand extends Command
    {
        public returnArmCommand()
        {
            addRequirements(AlgaeRemover.this);
        }

        @Override
        public void initialize()
        {
            io.returnArm(returningSpeed);
        }

        @Override
        public void execute()
        {
            if (!hasReachedTop())
            {
                io.returnArm(returningSpeed);
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
            io.stopMotor();
        }
    }

    public Command getRemoveAlgaeCommand(double speed)
    {
        return new removeAlgaeCommand();
    }

    public Command returnArmCommand(double speed)
    {
        return new returnArmCommand();
    }
}
