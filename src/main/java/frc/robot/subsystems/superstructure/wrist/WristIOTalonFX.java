package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import java.util.function.Supplier;

public class WristIOTalonFX implements WristIO
{
    private final TalonFX motor;
    private final CANcoder cancoder;
    private final StatusSignal<Angle> cancoderAngle;
    private final StatusSignal<Temperature> motorTemperature;
    private final StatusSignal<Current> motorCurrent;
    private final Supplier<Boolean> motorPresent;
    private Angle targetAngle;
    private MotionMagicVoltage motorControl;

    public WristIOTalonFX(int motorid, int cancoderid)
    {
        motor = new TalonFX(motorid);
        cancoder = new CANcoder(cancoderid);
        cancoderAngle = cancoder.getPosition();
        motorTemperature = motor.getDeviceTemp();
        motorCurrent = motor.getTorqueCurrent();
        motorPresent = () -> motor.isConnected();
        targetAngle = null;

        configureMotor(motorid, cancoderid);
    }

    @Override
    public void set(double speed)
    {
        motor.set(speed);
    }

    @Override
    public void moveToAngle(Angle angle)
    {
        targetAngle = angle;
        motorControl = new MotionMagicVoltage(0).withSlot(0);
        motor.setControl(motorControl.withPosition(angle));
    }

    @Override
    public void updateInputs(WristIOInputs inputs)
    {
        inputs.encoderAngle = cancoderAngle.getValue();
        inputs.motorTemperature = motorTemperature.getValue();
        inputs.motorCurrent = motorCurrent.getValue();
        inputs.motorPresent = motorPresent.get();
    }

    @Override
    public Angle getTargetAngle()
    {
        return targetAngle;
    }

    @Override
    public void resetEncoderAngle(Angle angle)
    {
        cancoder.setPosition(angle);
    }

    public void configureMotor(int motorid, int cancoderid)
    {
        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.Feedback.FeedbackRemoteSensorID = cancoderid;
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25;
        slot0Configs.kV = 0.12;
        slot0Configs.kP = 4.8;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.1;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 160;
        motionMagicConfigs.MotionMagicJerk = 1600;

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfigs.MagnetSensor.MagnetOffset = 0.4;

        talonFXConfigs.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        talonFXConfigs.Feedback.SensorToMechanismRatio = 1.0;
        talonFXConfigs.Feedback.RotorToSensorRatio = 12.8; // TODO

        motor.getConfigurator().apply(talonFXConfigs);
        cancoder.getConfigurator().apply(cancoderConfigs);

        motorControl = new MotionMagicVoltage(0).withSlot(0);
    }
}