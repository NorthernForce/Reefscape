package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;

public class IntakeIOTalonFX implements IntakeIO
{
	private final TalonFX intakeMotor;
	private final StatusSignal<Angle> position;

	public IntakeIOTalonFX(int id)
	{
		intakeMotor = new TalonFX(id);
		position = intakeMotor.getPosition();
	}

	@Override
	public void intake()
	{
		intakeMotor.set(0.5);
	}

	@Override
	public void outtake()
	{
		intakeMotor.set(-0.5);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs)
	{
		inputs.position = position.getValue();
	}
}
