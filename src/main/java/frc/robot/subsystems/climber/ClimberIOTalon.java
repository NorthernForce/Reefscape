package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;

public class ClimberIOTalon implements ClimberIO
{
	private TalonFX m_motor;
	private double m_gearRatio;
	private StatusSignal<Angle> m_position;

	public ClimberIOTalon(int id, double gearRatio)
	{
		m_motor = new TalonFX(id);
		m_gearRatio = gearRatio;
		m_position = m_motor.getPosition();
	}

	@Override
	public void climbUp()
	{
		m_motor.set(0.5);
	}

	@Override
	public void stop()
	{
		m_motor.stopMotor();
	}

	@Override
	public void climbDown()
	{
		m_motor.set(-0.5);
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs)
	{
		inputs.position = Rotations.of(m_position.getValue().in(Rotation) / m_gearRatio);
	}

}