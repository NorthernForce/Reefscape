package frc.robot.subsystems.specialstick.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeLimitSwitchIO implements AlgaeRemoverSensorIO
{
    DigitalInput switchInput;

    public AlgaeLimitSwitchIO(int port)
    {
        switchInput = new DigitalInput(port);
    }

    @Override
    public void updateInputs(AlgaeRemoverSensorIOInputs inputs)
    {
        inputs.reachedTop = !switchInput.get();
    }

}
