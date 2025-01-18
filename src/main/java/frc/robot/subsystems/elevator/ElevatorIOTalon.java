package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.zippy.constants.ZippyConstants;

public class ElevatorIOTalon implements ElevatorIO
{
    private TalonFX m_motor;
    private TalonFX m_innerMotor;
    private StatusSignal<Angle> m_position;
    private StatusSignal<Angle> m_innerPosition;
    private StatusSignal<Temperature> m_innerTemperature;
    private StatusSignal<Temperature> m_temperature;

    public ElevatorIOTalon(int index, int innerIndex)
    {
        m_motor = new TalonFX(index);
        m_innerMotor = new TalonFX(innerIndex);
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
    }

    @SuppressWarnings("removal")
    public void setInverted(boolean inverted)
    {
        m_motor.setInverted(inverted);
    }
    
    @SuppressWarnings("removal")
    public void setInnerInverted(boolean inverted)
    {
        m_innerMotor.setInverted(inverted);
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
        return Meters.of(m_position.getValueAsDouble() / ZippyConstants.ElevatorConstants.INNER_GEAR_RATIO
                * ZippyConstants.ElevatorConstants.INNER_SPROCKET_CIRCUMFERENCE);
    }

}
