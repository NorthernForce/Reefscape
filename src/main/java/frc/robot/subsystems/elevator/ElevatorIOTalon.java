package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.zippy.constants.ZippyConstants;

public class ElevatorIOTalon implements ElevatorIO
{
    private TalonFX m_motor;
    private TalonFX m_innerMotor;
    private TalonFXConfiguration talonFXConfigs;
    private TalonFXConfiguration talonFXInnerConfigs;
    private StatusSignal<Angle> m_position;
    private StatusSignal<Angle> m_innerPosition;
    private StatusSignal<Temperature> m_innerTemperature;
    private StatusSignal<Temperature> m_temperature;

    public ElevatorIOTalon(int index, int innerIndex, double kS, double kV, double kA, double kP, double kI, double kD,
            double cruiseVelocity, double acceleration, double jerk)
    {
        m_motor = new TalonFX(index);
        m_innerMotor = new TalonFX(innerIndex);

        talonFXConfigs = new TalonFXConfiguration();
        talonFXInnerConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        slot0Configs.kA = kA;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        var slot0InnerConfigs = talonFXInnerConfigs.Slot0;
        slot0InnerConfigs.kS = kS;
        slot0InnerConfigs.kV = kV;
        slot0InnerConfigs.kA = kA;
        slot0InnerConfigs.kP = kP;
        slot0InnerConfigs.kI = kI;
        slot0InnerConfigs.kD = kD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = acceleration;
        motionMagicConfigs.MotionMagicJerk = jerk;

        var motionMagicInnerConfigs = talonFXInnerConfigs.MotionMagic;
        motionMagicInnerConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
        motionMagicInnerConfigs.MotionMagicAcceleration = acceleration;
        motionMagicInnerConfigs.MotionMagicJerk = jerk;

        m_motor.getConfigurator().apply(talonFXConfigs);
        m_innerMotor.getConfigurator().apply(talonFXInnerConfigs);

        m_position = m_motor.getPosition();
        m_innerPosition = m_innerMotor.getPosition();
        m_temperature = m_motor.getDeviceTemp();
        m_innerTemperature = m_innerMotor.getDeviceTemp();
    }

    public void start(double speed)
    {
        m_motor.set(speed);
    }

    public void stop()
    {
        m_motor.stopMotor();
    }

    @Override
    public Distance getPosition()
    {
        return Meters.of(m_position.getValueAsDouble() / ZippyConstants.ElevatorConstants.GEAR_RATIO
                * ZippyConstants.ElevatorConstants.SPROCKET_CIRCUMFERENCE);
    }

    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs)
    {
        inputs.temperature = m_temperature.getValueAsDouble();
        inputs.innerTemperature = m_innerTemperature.getValueAsDouble();
        inputs.position = getPosition();
        inputs.innerPosition = getInnerPosition();
    }

    public void setInverted(boolean inverted)
    {
        talonFXConfigs.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        m_motor.getConfigurator().apply(talonFXConfigs);
    }

    public void setInnerInverted(boolean inverted)
    {
        talonFXInnerConfigs.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        m_innerMotor.getConfigurator().apply(talonFXInnerConfigs);
    }

    @Override
    public void startInner(double speed)
    {
        m_innerMotor.set(speed);
    }

    @Override
    public void stopInner()
    {
        m_innerMotor.stopMotor();
    }

    @Override
    public Distance getInnerPosition()
    {
        return Meters.of(m_innerPosition.getValueAsDouble() / ZippyConstants.ElevatorConstants.INNER_GEAR_RATIO
                * ZippyConstants.ElevatorConstants.INNER_SPROCKET_CIRCUMFERENCE);
    }

}
