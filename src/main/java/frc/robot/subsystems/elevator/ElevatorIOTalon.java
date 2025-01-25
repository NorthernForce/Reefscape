package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.zippy.constants.ZippyConstants.ElevatorConstants.ElevatorState;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

public class ElevatorIOTalon implements ElevatorIO
{
	private TalonFX m_motor;
	private TalonFXConfiguration talonFXConfigs;
	private StatusSignal<Angle> m_position;
	private StatusSignal<Temperature> m_temperature;
	private double gearRatio;
	private ElevatorState goingTo;
	private double errorTolerance;
	private Relay m_relay;

	public ElevatorIOTalon(int index, double kS, double kV, double kA, double kP, double kI, double kD,
			double cruiseVelocity, double acceleration, double jerk, double errorTolerance,
			double sprocketCircumference, double gearRatio, int relayChannel)
	{
		m_motor = new TalonFX(index);
		this.errorTolerance = errorTolerance;
		talonFXConfigs = new TalonFXConfiguration();
		this.gearRatio = gearRatio;
		m_relay = new Relay(relayChannel);

		var slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.kS = kS;
		slot0Configs.kV = kV;
		slot0Configs.kA = kA;
		slot0Configs.kP = kP;
		slot0Configs.kI = kI;
		slot0Configs.kD = kD;

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
		motionMagicConfigs.MotionMagicAcceleration = acceleration;
		motionMagicConfigs.MotionMagicJerk = jerk;

		m_motor.getConfigurator().apply(talonFXConfigs);

		m_position = m_motor.getPosition();
		m_temperature = m_motor.getDeviceTemp();
	}

	@Override
	public void setTargetPosition(double speed, ElevatorState level)
	{
		goingTo = level;
		m_relay.set(Value.kOn);
		m_motor.setControl(new MotionMagicVoltage(gearRatio * level.getHeight()));
	}

	public void stop()
	{
		m_relay.set(Value.kOff);
		m_motor.stopMotor();
	}

	@Override
	public void updateInputs(ElevatorIO.ElevatorIOInputs inputs)
	{
		inputs.temperature = m_temperature.getValueAsDouble();
		inputs.position = Meters.of(m_position.getValueAsDouble() / gearRatio);
		inputs.isAtTargetPosition = Math.abs(inputs.position.in(Meters) - goingTo.getHeight()) < errorTolerance;
	}

	public void setInverted(boolean inverted)
	{
		m_relay.setDirection((inverted ? Direction.kReverse : Direction.kForward));
	}

}
