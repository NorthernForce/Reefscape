package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;

import java.util.function.Supplier;

public class WristIOTalonFX implements WristIO
{
    private final TalonFX motor;
    private final CANcoder cancoder;
    private final StatusSignal<Angle> cancoderAngle;
    private final StatusSignal<Temperature> motorTemperature;
    private final StatusSignal<Current> motorCurrent;

    private final Supplier<Boolean> motorPresent;

    public WristIOTalonFX(int motorid, int cancoderid)
    {
        motor = new TalonFX(motorid);
        cancoder = new CANcoder(cancoderid);
        cancoderAngle = cancoder.getPosition();
        motorTemperature = motor.getDeviceTemp();
        motorCurrent = motor.getTorqueCurrent();
        motorPresent = () -> motor.isConnected();
    }

    @Override
    public void run(double speed)
    {
        motor.set(speed);
    }

    @Override
    public void runToAngle(Rotation2d angle)
    {

    }

    @Override
    public void updateInputs(WristIOInputs inputs)
    {
        inputs.encoderAngle = cancoder.getPosition().getValue();
        inputs.motorTemperature = motorTemperature.getValue();
        inputs.motorCurrent = motorCurrent.getValue();
        inputs.motorPresent = motorPresent.get();
    }
}