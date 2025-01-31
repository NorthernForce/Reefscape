package frc.robot.subsystems.elevator;

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
    private Distance goingTo;
    private final double errorTolerance;

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
     * @param upperLimit            the upper limit of the elevator
     */
    public ElevatorIOTalonFX(int id, double kS, double kV, double kA, double kP, double kI, double kD,
            LinearVelocity cruiseVelocity, LinearAcceleration acceleration, double jerk, double errorTolerance,
            Distance sprocketCircumference, double gearRatio, boolean inverted, Distance upperLimit)
    {
        m_motor = new TalonFX(id);
        this.errorTolerance = errorTolerance;
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
        inputs.isAtTargetPosition = Math.abs(inputs.position.in(Meters) - goingTo.in(Meters)) < errorTolerance;
        inputs.current = m_current.getValue();
        inputs.present = m_isPresent.get();
    }

}
