package frc.robot.subsystems.oculus;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Oculus extends SubsystemBase
{
    private final OculusIO io;
    private final OculusIOInputsAutoLogged inputs = new OculusIOInputsAutoLogged();
	public Oculus(OculusIO io)
    {
        this.io = io;
    }

	@Override
	public void periodic()
	{
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    public boolean isConnected()
    {
        return inputs.connected;
    }
}
