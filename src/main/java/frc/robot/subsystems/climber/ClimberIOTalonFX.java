package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

/**
 * IO for the climber using a TalonFX motor controller.
 */

public class ClimberIOTalonFX implements ClimberIO
{
	private TalonFX m_motor;
	private StatusSignal<Angle> m_position;
	private Supplier<Boolean> m_present;
	private StatusSignal<Temperature> m_temperature;
	private StatusSignal<Current> m_current;
	private CANcoder m_encoder;

	/**
	 * Constructor for the ClimberIOTalonFX class.
	 * 
	 * @param id        motor controller ID
	 * @param inverted  whether the motor is inverted
	 * @param encoderID CANcoder ID
	 */

	public ClimberIOTalonFX(int id, boolean inverted, int encoderID)
	{
		m_encoder = new CANcoder(encoderID);
		m_motor = new TalonFX(id);
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.Inverted = (inverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive);
		config.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
		config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		m_motor.getConfigurator().refresh(config);
		m_position = m_encoder.getAbsolutePosition();
		m_present = () -> m_motor.isConnected();
		m_temperature = m_motor.getDeviceTemp();
		m_current = m_motor.getSupplyCurrent();
	}

	/**
	 * climb up method for the ClimberIOTalonFX class.
	 * 
	 * @param climbSpeed speed to climb up
	 */

	@Override
	public void climbUp(double climbSpeed)
	{
		m_motor.set(Math.abs(climbSpeed));
	}

	/**
	 * stop method for the ClimberIOTalonFX class.
	 */

	@Override
	public void stop()
	{
		m_motor.stopMotor();
	}

	/**
	 * climb down method for the ClimberIOTalonFX class.
	 * 
	 * @param climbSpeed speed to climb down
	 */

	@Override
	public void climbDown(double climbSpeed)
	{
		m_motor.set(-Math.abs(climbSpeed));
	}

	/**
	 * update inputs method for the ClimberIOTalonFX class.
	 * 
	 * @param inputs ClimberIOInputs inputs to update
	 */

	@Override
	public void updateInputs(ClimberIOInputs inputs)
	{
		inputs.position = Rotations.of(m_position.getValue().in(Rotation));
		inputs.current = m_current.getValue();
		inputs.present = m_present.get();
		inputs.temperature = m_temperature.getValue();
	}

}