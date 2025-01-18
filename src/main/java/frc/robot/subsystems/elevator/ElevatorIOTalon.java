package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.zippy.constants.ZippyConstants;

public class ElevatorIOTalon implements ElevatorIO
{
	@AutoLog
	public static class ElevatorIOInputs
	{
		public double temperature = 0;
		public ElevatorState state = ElevatorState.L1;
	}

	private TalonFX m_motor;
	private StatusSignal<Angle> m_position;
	private StatusSignal<Temperature> m_temperature;

	public ElevatorIOTalon(int index)
	{
		m_motor = new TalonFX(index);
		m_position = m_motor.getPosition();
		m_temperature = m_motor.getDeviceTemp();
	}

	public void start(double speed)
	{
		m_motor.set(speed);
	}

	public void stop()
	{
		m_motor.stopMotor();
	}

	@Override
	public Distance getPosition()
	{
		return Meters.of(m_position.getValueAsDouble() / ZippyConstants.ElevatorConstants.GEAR_RATIO
				* ZippyConstants.ElevatorConstants.SPROCKET_CIRCUMFERENCE);
	}

	@Override
	public void updateInputs(ElevatorIO.ElevatorIOInputs inputs)
	{
		inputs.temperature = m_temperature.getValueAsDouble();
    }

	@SuppressWarnings("removal")
	public void setInverted(boolean inverted)
	{
		m_motor.setInverted(inverted);
	}

}
