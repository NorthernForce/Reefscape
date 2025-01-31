package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;

/**
 * ElevatorIOTalon is a class that implements ElevatorIO using a TalonFX.
 */

public class ElevatorIOTalonFX implements ElevatorIO
{
    private final TalonFX m_motor;
    private final StatusSignal<Angle> m_position;
    private final StatusSignal<Temperature> m_temperature;
    private final StatusSignal<Current> m_current;
    private final Supplier<Boolean> m_isPresent;
    private final double gearRatio;
    private final Distance sprocketCircumference;
    private final MotionMagicVoltage m_motionMagicVoltage;

    /**
     * Converts rotations to distance
     * 
     * @param rotations the rotations to convert
     * @return the distance
     */
    private final Distance convertRotationsToDistance(Angle rotations)
    {
        return Meters.of(rotations.in(Rotations) / gearRatio * sprocketCircumference.in(Meters));
    }

    /**
     * Converts distance to rotations
     * 
     * @param distance the distance to convert
     * @return the rotations
     */
    private final Angle convertDistanceToRotations(Distance distance)
    {
        return Rotations.of(distance.in(Meters) / sprocketCircumference.in(Meters) * gearRatio);
    }

    /**
     * Converts linear velocity to angular velocity
     * 
     * @param linearVelocity the linear velocity to convert
     * @return the angular velocity
     */
    private final AngularVelocity convertLinearVelocityToAngularVelocity(LinearVelocity linearVelocity)
    {
        return RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) * gearRatio / sprocketCircumference.in(Meters));
    }

    /**
     * Converts linear acceleration to angular acceleration
     * 
     * @param linearAcceleration the linear acceleration to convert
     * @return the angular acceleration
     */
    private final AngularAcceleration convertLinearAccelerationToAngularAcceleration(
            LinearAcceleration linearAcceleration)
    {
        return RotationsPerSecondPerSecond
                .of(linearAcceleration.in(MetersPerSecondPerSecond) * gearRatio / sprocketCircumference.in(Meters));
    }

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
    public static record ElevatorConstants(double kS, double kV, double kA, double kP, double kI, double kD,
            LinearVelocity cruiseVelocity, LinearAcceleration acceleration, double jerk, Distance sprocketCircumference,
            double gearRatio, boolean inverted, Distance upperLimit) {
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
                constants.cruiseVelocity(), constants.acceleration(), constants.jerk(),
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
    public ElevatorIOTalonFX(int id, double kS, double kV, double kA, double kP, double kI, double kD,
            LinearVelocity cruiseVelocity, LinearAcceleration acceleration, double jerk, Distance sprocketCircumference,
            double gearRatio, boolean inverted, Distance upperLimit)
    {
        m_motor = new TalonFX(id);
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        this.gearRatio = gearRatio;
        this.sprocketCircumference = sprocketCircumference;

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        slot0Configs.kA = kA;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = convertLinearVelocityToAngularVelocity(cruiseVelocity)
                .in(RotationsPerSecond);
        motionMagicConfigs.MotionMagicAcceleration = convertLinearAccelerationToAngularAcceleration(acceleration)
                .in(RotationsPerSecondPerSecond);
        motionMagicConfigs.MotionMagicJerk = jerk;
        talonFXConfigs.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = convertDistanceToRotations(upperLimit)
                .in(Rotations);
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        m_motor.getConfigurator().apply(talonFXConfigs);

        m_position = m_motor.getPosition();
        m_temperature = m_motor.getDeviceTemp();
        m_current = m_motor.getTorqueCurrent();
        m_isPresent = () -> m_motor.isConnected();

        m_motionMagicVoltage = new MotionMagicVoltage(0);
    }

    /**
     * Sets the target position of the elevator
     * 
     * @param height the height to set the elevator to
     */

    @Override
    public void setTargetPosition(Distance height)
    {
        m_motor.setControl(m_motionMagicVoltage.withPosition(convertDistanceToRotations(height)));
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
        inputs.temperature = m_temperature.getValue();
        inputs.position = convertRotationsToDistance(m_position.getValue());
        inputs.current = m_current.getValue();
        inputs.present = m_isPresent.get();
    }

}
