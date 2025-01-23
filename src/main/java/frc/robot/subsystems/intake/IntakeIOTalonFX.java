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
    public void set(double speed)
    {
        intakeMotor.set(speed);
    }

	@Override
	public void updateInputs(IntakeIOInputs inputs)
	{
		inputs.position = position.getValue();
	}
}
