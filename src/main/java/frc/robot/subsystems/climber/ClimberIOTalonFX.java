package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
     * @param id         id of the motor controller
     * @param inverted   whether the motor controller is inverted
     * @param encoderID  id of the encoder
     * @param lowerLimit lower limit of the climber
     * @param upperLimit upper limit of the climber
     */

    public ClimberIOTalonFX(int id, boolean inverted, int encoderID, Angle lowerLimit, Angle upperLimit)
    {
        m_encoder = new CANcoder(encoderID);
        m_motor = new TalonFX(id);
        TalonFXConfiguration config = new TalonFXConfiguration();
        CANcoderConfiguration cc = new CANcoderConfiguration();
        cc.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
        config.MotorOutput.Inverted = (inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        if (m_encoder.isConnected())
        {
            config.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = upperLimit.in(Rotations);
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = lowerLimit.in(Rotations);
        }

        m_motor.getConfigurator().apply(config);
        m_position = m_encoder.getAbsolutePosition();
        m_present = () -> m_motor.isConnected();
        m_temperature = m_motor.getDeviceTemp();
        m_current = m_motor.getSupplyCurrent();
    }

    /**
     * run method for the ClimberIOTalonFX class at a certain speed.
     * 
     * @param speed speed to run the climber at
     */

    @Override
    public void run(double speed)
    {
        m_motor.set(speed);
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
     * update inputs method for the ClimberIOTalonFX class.
     * 
     * @param inputs ClimberIOInputs inputs to update
     */

    @Override
    public void updateInputs(ClimberIOInputs inputs)
    {
        BaseStatusSignal.refreshAll(m_position, m_current, m_temperature);
        inputs.position = m_position.getValue();
        inputs.current = m_current.getValue();
        inputs.present = m_present.get();
        inputs.temperature = m_temperature.getValue();
    }

}