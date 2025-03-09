package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;

/**
 * ElevatorIOTalon is a class that implements ElevatorIO using a TalonFX.
 */

public class ElevatorIOTalonFX implements ElevatorIO
{
    private final TalonFX m_motor;
    private final StatusSignal<Angle> m_position;
    private final StatusSignal<Temperature> m_temperature;
    private final StatusSignal<Voltage> m_voltage;
    private final StatusSignal<Current> m_current;
    private final StatusSignal<AngularVelocity> m_velocity;
    private final StatusSignal<AngularVelocity> m_rotorVelocity;
    private final Supplier<Boolean> m_isPresent;
    private final MotionMagicExpoVoltage m_motionMagicVoltage;
    private final DutyCycleOut m_duty = new DutyCycleOut(0);
    private final VoltageOut m_voltageOut = new VoltageOut(0);
    private final double kG;

    /**
     * Constants for the elevator
     * 
     * @param kS                    the kS value
     * @param kV                    the kV value
     * @param kA                    the kA value
     * @param kP                    the kP value
     * @param kI                    the kI value
     * @param kD                    the kD value
     * @param cruiseVelocity        the cruise velocity
     * @param acceleration          the acceleration
     * @param jerk                  the jerk
     * @param sprocketCircumference the sprocket circumference
     * @param gearRatio             the gear ratio
     * @param inverted              whether the motor is inverted
     * @param upperLimit            the upper limit
     */
    public static record ElevatorConstants(double kS, double kV, double kA, double kP, double kI, double kD, double kG,
            double cruiseVelocity, double acceleration, double jerk, Distance sprocketCircumference, double gearRatio,
            boolean inverted, Distance upperLimit) {
    }

    /**
     * Creates a new ElevatorIOTalonFX
     * 
     * @param id        the id of the talon
     * @param constants the constants for the elevator
     */
    public ElevatorIOTalonFX(int id, ElevatorConstants constants)
    {
        this(id, constants.kS(), constants.kV(), constants.kA(), constants.kP(), constants.kI(), constants.kD(),
                constants.kG(), constants.cruiseVelocity(), constants.acceleration(), constants.jerk(),
                constants.sprocketCircumference(), constants.gearRatio(), constants.inverted(), constants.upperLimit());
    }

    /**
     * Creates a new ElevatorIOTalonFX
     * 
     * @param id                    the id of the talon
     * @param kS                    the kS value
     * @param kV                    the kV value
     * @param kA                    the kA value
     * @param kP                    the kP value
     * @param kI                    the kI value
     * @param kD                    the kD value
     * @param cruiseVelocity        the cruise velocity
     * @param acceleration          the acceleration
     * @param jerk                  the jerk
     * @param sprocketCircumference the sprocket circumference
     * @param gearRatio             the gear ratio
     * @param inverted              whether the motor is inverted
     * @param upperLimit            the upper limit
     */
    public ElevatorIOTalonFX(int id, double kS, double kV, double kA, double kP, double kI, double kD, double kG,
            double cruiseVelocity, double acceleration, double jerk, Distance sprocketCircumference, double gearRatio,
            boolean inverted, Distance upperLimit)
    {
        m_motor = new TalonFX(id);
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        slot0Configs.kA = kA;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        slot0Configs.kG = kG;
        this.kG = kG;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = motionMagicConfigs.MotionMagicAcceleration = 160;
        motionMagicConfigs.MotionMagicJerk = jerk;

        talonFXConfigs.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonFXConfigs.Feedback.RotorToSensorRatio = 1;
        talonFXConfigs.Feedback.SensorToMechanismRatio = gearRatio / sprocketCircumference.in(Inches);

        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = upperLimit.in(Inches);
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 40;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        m_motor.getConfigurator().apply(talonFXConfigs);

        m_position = m_motor.getPosition();
        m_temperature = m_motor.getDeviceTemp();
        m_current = m_motor.getTorqueCurrent();
        m_velocity = m_motor.getVelocity();
        m_rotorVelocity = m_motor.getRotorVelocity();
        m_voltage = m_motor.getMotorVoltage();
        m_isPresent = () -> m_motor.isConnected();

        m_motionMagicVoltage = new MotionMagicExpoVoltage(0);
    }

    /**
     * Sets the target position of the elevator
     * 
     * @param height the height to set the elevator to
     */

    @Override
    public void setTargetPosition(Distance height)
    {
        m_motor.setControl(m_motionMagicVoltage.withPosition(height.in(Inches)));
    }

    @Override
    public void setLowerLimitEnable(boolean enableLowerLimit)
    {

        TalonFXConfiguration config = new TalonFXConfiguration();
        m_motor.getConfigurator().refresh(config);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enableLowerLimit;
        m_motor.getConfigurator().apply(config);
    }

    @Override
    public void setSpeed(double speed, boolean overrideLowerLimit)
    {
        m_motor.setControl(m_duty.withOutput(speed + kG / RobotController.getInputVoltage()));
    }

    /**
     * Resets the position of the elevator
     */
    @Override
    public void resetPosition()
    {
        m_motor.setPosition(0);
    }

    /**
     * Stops the elevator motor
     */
    @Override
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
        BaseStatusSignal.refreshAll(m_temperature, m_position, m_current, m_velocity, m_rotorVelocity, m_voltage);
        inputs.temperature = m_temperature.getValue();
        inputs.position = Inches.of(m_position.getValue().in(Rotations));
        inputs.current = m_current.getValue();
        inputs.present = m_isPresent.get();
        inputs.velocity = InchesPerSecond.of(m_velocity.getValue().in(RotationsPerSecond));
        inputs.rotorVelocity = m_rotorVelocity.getValue();
        inputs.voltage = m_voltage.getValue();
    }

    @Override
    public void setVoltage(Voltage voltage)
    {
        m_motor.setControl(m_voltageOut.withOutput(voltage));
    }

}
