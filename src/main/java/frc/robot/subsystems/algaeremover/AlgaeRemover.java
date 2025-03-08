package frc.robot.subsystems.algaeremover;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeremover.sensor.AlgaeSensorIO;

public class AlgaeRemover extends SubsystemBase
{
    private AlgaeRemoverIO io;
    private AlgaeSensorIO sensorIO;
    private AlgaeSensorIO.AlgaeSensorInputs inputs = new AlgaeSensorIO.AlgaeSensorInputs();
    private AlgaeRemoverIO.AlgaeRemoverIOInputs algaeRemoverInputs = new AlgaeRemoverIO.AlgaeRemoverIOInputs();

    public AlgaeRemover(AlgaeRemoverIO algaeRemoverIO, AlgaeSensorIO algaeSensorIO)
    {
        io = algaeRemoverIO;
        sensorIO = algaeSensorIO;
    }

    public void removeAlgae(double speed)
    {
        io.removeAlgae(speed);
    }

    public void returnArm(double speed)
    {
        io.returnArm(speed);
    }
    
    @AutoLogOutput
    public boolean hasReachedTop()
    {
        return inputs.reachedTop;
    }

    public void algaeRemoved(boolean algaeRemoved)
    {
        io.algaeRemoved(algaeRemoved);
    }

    public boolean getAlgaeRemoved()
    {
        return io.getAlgaeRemoved();
    }

    public void stopMotor() {
        io.stopMotor();
    }

    @Override
    public void periodic()
    {
        sensorIO.updateInputs(inputs);
        io.updateInputs(algaeRemoverInputs);
    }

    public Command removeAlgaeCommand(double speed)
    {
        return run(() -> removeAlgae(speed)).until(() -> getAlgaeRemoved()).andThen(runOnce(() -> stopMotor()));
    }

    public Command returnArmCommand(double speed)
    {
        return run(() -> returnArm(speed)).until(() -> hasReachedTop()).andThen(runOnce(() -> stopMotor()));
    }
}
