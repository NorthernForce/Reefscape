package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;

public class ElevatorIOTalon implements ElevatorIO
{
	private TalonFX m_motor;
	private TalonFXConfiguration talonFXConfigs;
	private StatusSignal<Angle> m_position;
	private StatusSignal<Temperature> m_temperature;
	private double gearRatio;
	private ElevatorState goingTo;
	private double errorTollerance;

	public ElevatorIOTalon(int index, double kS, double kV, double kA, double kP, double kI, double kD,
			double cruiseVelocity, double acceleration, double jerk, double errorTollerance,
			double sprocketCircumference, double gearRatio)
	{
		m_motor = new TalonFX(index);
		this.errorTollerance = errorTollerance;
		talonFXConfigs = new TalonFXConfiguration();
		this.gearRatio = gearRatio;

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

	public void start(double speed, ElevatorState level)
	{
		goingTo = level;
		m_motor.setControl(new MotionMagicVoltage(gearRatio * level.getHeight()));
	}

	public void stop()
	{
		m_motor.stopMotor();
	}

	@Override
	public void updateInputs(ElevatorIO.ElevatorIOInputs inputs)
	{
		inputs.temperature = m_temperature.getValueAsDouble();
		inputs.position = Meters.of(m_position.getValueAsDouble() / gearRatio);
		inputs.isAtTargetPosition = Math.abs(inputs.position.in(Meters) - goingTo.getHeight()) < errorTollerance;
		inputs.goingTo = goingTo;
	}

	public void setInverted(boolean inverted)
	{
		talonFXConfigs.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		m_motor.getConfigurator().apply(talonFXConfigs);
	}

}
