package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;

/**
 * ElevatorIOTalon is a class that implements ElevatorIO using a TalonFX.
 */

public class ElevatorIOTalon implements ElevatorIO
{
	private TalonFX m_motor;
	private TalonFXConfiguration talonFXConfigs;
	private StatusSignal<Angle> m_position;
	private StatusSignal<Temperature> m_temperature;
	private StatusSignal<Current> m_current;
	private Supplier<Boolean> m_isPresent;
	private double gearRatio;
	private Distance goingTo;
	private double errorTolerance;

	/**
	 * Creates a new ElevatorIOTalon
	 * 
	 * @param id                    the id of the talon
	 * @param kS                    kS parameter for the talon configs
	 * @param kV                    kV parameter for the talon configs
	 * @param kA                    kA parameter for the talon configs
	 * @param kP                    kP parameter for the talon configs
	 * @param kI                    kI parameter for the talon configs
	 * @param kD                    kD parameter for the talon configs
	 * @param cruiseVelocity        cruise velocity for the talon configs
	 * @param acceleration          acceleration for the talon configs
	 * @param jerk                  jerk movement for the talon configs
	 * @param errorTolerance        error tolerance for the elevator to be at the
	 *                              target position
	 * @param sprocketCircumference sprocket circumference
	 * @param gearRatio             gear ratio
	 * @param inverted              true if the talon is inverted, false if the
	 *                              talon is not inverted
	 */

	public ElevatorIOTalon(int id, double kS, double kV, double kA, double kP, double kI, double kD,
			double cruiseVelocity, double acceleration, double jerk, double errorTolerance,
			double sprocketCircumference, double gearRatio, boolean inverted)
	{
		m_motor = new TalonFX(id);
		this.errorTolerance = errorTolerance;
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
		talonFXConfigs.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;

		m_motor.getConfigurator().apply(talonFXConfigs);

		m_position = m_motor.getPosition();
		m_temperature = m_motor.getDeviceTemp();
		m_current = m_motor.getTorqueCurrent();
		m_isPresent = () -> m_motor.isConnected();
	}

	/**
	 * Sets the target position of the elevator
	 * 
	 * @param height the height to set the elevator to
	 */

	@Override
	public void setTargetPosition(double height)
	{
		goingTo = Meters.of(height);
		m_motor.setControl(new MotionMagicVoltage(gearRatio * height));
	}

	/**
	 * Stops the elevator motor
	 */

	public void stop()
	{
		m_motor.stopMotor();
	}

	/**
	 * Updates the inputs for the elevator
	 * 
	 * @param inputs the inputs to update
	 */

	@Override
	public void updateInputs(ElevatorIO.ElevatorIOInputs inputs)
	{
		inputs.temperature = m_temperature.getValue();
		inputs.position = Meters.of(m_position.getValueAsDouble() / gearRatio);
		inputs.isAtTargetPosition = Math.abs(inputs.position.in(Meters) - goingTo.in(Meters)) < errorTolerance;
		inputs.current = m_current.getValue();
		inputs.present = m_isPresent.get();
	}

}
