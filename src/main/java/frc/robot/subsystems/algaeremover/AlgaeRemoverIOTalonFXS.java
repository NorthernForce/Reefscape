package frc.robot.subsystems.algaeremover;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class AlgaeRemoverIOTalonFXS implements AlgaeRemoverIO
{
    private final TalonFXS talonFXS;

    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorCurrent;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Boolean> hallSensorFault;

    public AlgaeRemoverIOTalonFXS(int motorID, boolean inverted, double gearRatio)
    {
        talonFXS = new TalonFXS(motorID);
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.ExternalFeedback.SensorToMechanismRatio = gearRatio;
        talonFXS.getConfigurator().apply(config);
        motorVoltage = talonFXS.getMotorVoltage();
        motorCurrent = talonFXS.getStatorCurrent();
        position = talonFXS.getPosition();
        velocity = talonFXS.getVelocity();
        temperature = talonFXS.getDeviceTemp();
        hallSensorFault = talonFXS.getFault_HallSensorMissing();
    }

    @Override
    public void set(double speed)
    {
        talonFXS.set(speed);
    }

    @Override
    public void stopMotor()
    {
        talonFXS.stopMotor();
    }

    @Override
    public void updateInputs(AlgaeRemoverIOInputs inputs)
    {
        BaseStatusSignal.refreshAll(motorVoltage, motorCurrent, position, velocity, temperature, hallSensorFault);
        inputs.isPresent = talonFXS.isConnected() && !hallSensorFault.getValue();
        inputs.current = motorCurrent.getValue();
        inputs.voltage = motorVoltage.getValue();
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.temperature = temperature.getValue();
    }
}
