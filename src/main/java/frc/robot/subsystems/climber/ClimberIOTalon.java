package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberIOTalon implements ClimberIO
{
	private TalonFX m_motor;

	public ClimberIOTalon(TalonFX talon)
	{
		m_motor = talon;
	}

	@Override
	public void climbUpShallow()
	{
		m_motor.set(0.5);
	}

	@Override
	public void stop()
	{
		m_motor.stopMotor();
	}

	@Override
	public void climbDownShallow()
	{
		m_motor.set(-0.5);
	}

}
