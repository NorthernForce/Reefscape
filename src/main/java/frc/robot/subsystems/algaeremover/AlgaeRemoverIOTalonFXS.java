package frc.robot.subsystems.algaeremover;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeRemoverIOTalonFXS implements AlgaeRemoverIO {
    // find actual angle later
    private boolean algaeRemoved = false;
    private TalonFXS talonFXS;

    public AlgaeRemoverIOTalonFXS(int motorID)
    {
        talonFXS = new TalonFXS(motorID);
    }

    @Override
    public void removeAlgae(double speed)
    {
        talonFXS.set(-speed);
    }

    @Override
    public void returnArm(double speed)
    {
        talonFXS.set(speed);
    }

    @Override
    public void algaeRemoved(boolean algaeRemoved)
    {
        algaeRemoved = true;
    }

    @Override
    public boolean getAlgaeRemoved()
    {
        return algaeRemoved;
    }

    @Override
    public void stopMotor()
    {
        talonFXS.stopMotor();
    }

    @Override
    public void updateInputs(AlgaeRemoverIOInputs inputs)
    {
        inputs.algaeRemoved = algaeRemoved;
    }
}
