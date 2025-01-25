package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Temperature;

public class RollersIOTalonFX implements RollersIO
{
	private final TalonFX intakeMotor;
	private final StatusSignal<Temperature> temperature;

	public RollersIOTalonFX(int id)
	{
		intakeMotor = new TalonFX(id);
		temperature = intakeMotor.getDeviceTemp();
	}

	@Override
	public void set(double speed)
	{
		intakeMotor.set(speed);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs)
	{
		inputs.temperature = temperature.getValue();
	}
}
