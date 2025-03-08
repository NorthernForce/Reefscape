package frc.robot.subsystems.algaeremover.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeBeamBreakIO implements AlgaeSensorIO {
    DigitalInput beamBreak;

    public AlgaeBeamBreakIO(int port) {
        beamBreak = new DigitalInput(port);
    }

    @Override
    public void updateInputs(AlgaeSensorInputs inputs) {
        inputs.reachedTop = !beamBreak.get();
    }
    
}
