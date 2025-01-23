package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotation;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberIOTalon implements ClimberIO
{
	private TalonFX m_motor;
	private double m_gearRatio;

	public ClimberIOTalon(int id, double gearRatio)
	{
		m_motor = new TalonFX(id);
		m_gearRatio = gearRatio;
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
		inputs.position = Degrees.of(m_motor.getPosition().getValue().in(Rotation) / m_gearRatio);
	}

}