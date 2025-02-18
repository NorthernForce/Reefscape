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

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

/**
 * WristIO to be used with a TalonFX motor controller
 */
public class WristIOTalonFX implements WristIO
{
    private final TalonFX motor;
    private final CANcoder cancoder;
    private final StatusSignal<Angle> cancoderAngle;
    private final StatusSignal<Temperature> motorTemperature;
    private final StatusSignal<Current> motorCurrent;
    private final Supplier<Boolean> motorPresent;
    private Angle targetAngle = Degrees.of(0);
    private MotionMagicVoltage motorControl;

    /**
     * Creates a new WristIOTalonFX
     * 
     * @param motorid    The CAN id of the wrist motor
     * @param cancoderid The CAN id of the wrist cancoder
     */
    public WristIOTalonFX(int motorid, int cancoderid, WristConstants wristConstants)
    {
        motor = new TalonFX(motorid);
        cancoder = new CANcoder(cancoderid);
        cancoderAngle = cancoder.getPosition();
        motorTemperature = motor.getDeviceTemp();
        motorCurrent = motor.getTorqueCurrent();
        motorPresent = () -> motor.isConnected();
        targetAngle = Degrees.of(0);

        configureMotor(motorid, cancoderid, wristConstants);
    }

    /**
     * Sets the wrist to run at the desired speed
     * 
     * @param speed (0.0 - 1.0) The speed to run the motor at
     */
    @Override
    public void set(double speed)
    {
        motor.set(speed);
    }

    /**
     * Moves the wrist to the desired angle using Motion Magic
     * 
     * @param angle The angle to move the wrist to
     */
    @Override
    public void moveToAngle(Angle angle)
    {
        targetAngle = angle;
        motorControl = new MotionMagicVoltage(0).withSlot(0);
        motor.setControl(motorControl.withPosition(angle));
    }

    /**
     * Updates the inputs of the wrist
     * 
     * @param inputs The inputs to update
     */
    @Override
    public void updateInputs(WristIOInputs inputs)
    {
        inputs.encoderAngle = cancoderAngle.getValue();
        inputs.targetAngle = targetAngle;
        inputs.motorTemperature = motorTemperature.getValue();
        inputs.motorCurrent = motorCurrent.getValue();
        inputs.motorPresent = motorPresent.get();
    }

    /**
     * Sets the current angle of the motor's cancoder to the inputted angle
     * 
     * @param angle The angle to set to
     */
    @Override
    public void resetEncoderAngle(Angle angle)
    {
        cancoder.setPosition(angle);
    }

    /**
     * Configures the TalonFX and CANcoder
     * 
     * @param motorid        The CAN id of the wrist motor
     * @param cancoderid     The CAN id of the wrist cancoder
     * @param wristConstants The constants for the wrist
     */
    public void configureMotor(int motorid, int cancoderid, WristConstants constants)
    {
        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.Feedback.FeedbackRemoteSensorID = cancoderid;
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = constants.kS();
        slot0Configs.kV = constants.kV();
        slot0Configs.kP = constants.kP();
        slot0Configs.kI = constants.kI();
        slot0Configs.kD = constants.kD();

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = constants.cruiseVelocity();
        motionMagicConfigs.MotionMagicAcceleration = constants.acceleration();
        motionMagicConfigs.MotionMagicJerk = constants.jerk();

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfigs.MagnetSensor.MagnetOffset = 0.4;

        talonFXConfigs.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        talonFXConfigs.Feedback.SensorToMechanismRatio = constants.sensorToMechanismRatio();
        talonFXConfigs.Feedback.RotorToSensorRatio = constants.rotorToSensorRatio();

        motor.getConfigurator().apply(talonFXConfigs);
        cancoder.getConfigurator().apply(cancoderConfigs);

        motorControl = new MotionMagicVoltage(0).withSlot(0);
    }

    public static record WristConstants(double kS, double kV, double kA, double kP, double kI, double kD,
            double cruiseVelocity, double acceleration, double jerk, boolean inverted, Angle upperLimit,
            Angle lowerLimit, double sensorToMechanismRatio, double rotorToSensorRatio) {
    }
}