package frc.robot.subsystems.specialstick.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeBeamBreakIO implements AlgaeRemoverSensorIO
{
    DigitalInput beamBreak;

    public AlgaeBeamBreakIO(int port)
    {
        beamBreak = new DigitalInput(port);
    }

    @Override
    public void updateInputs(AlgaeRemoverSensorIOInputs inputs)
    {
        inputs.reachedTop = !beamBreak.get();
    }

}
