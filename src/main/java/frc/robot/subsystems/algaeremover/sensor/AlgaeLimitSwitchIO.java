package frc.robot.subsystems.algaeremover.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeLimitSwitchIO implements AlgaeSensorIO {
    DigitalInput switchInput;
    public AlgaeLimitSwitchIO(int port) {
        switchInput = new DigitalInput(port);
    }

    @Override
    public void updateInputs(AlgaeSensorInputs inputs) {
        inputs.reachedTop = switchInput.get();
    }
    
}
