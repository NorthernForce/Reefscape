package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
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

        var talonFXConfigs = new TalonFXConfiguration();

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

        motor.getConfigurator().apply(talonFXConfigs);

        motorControl = new MotionMagicVoltage(0).withSlot(0);
    }

    @Override
    public void set(double speed)
    {
        motor.set(speed);
    }

    @Override
    public Command getMoveToAngleCommand(Angle angle)
    {
        targetAngle = angle;
        motorControl = new MotionMagicVoltage(0).withSlot(0);
        return Commands.runOnce(() -> motor.setControl(motorControl.withPosition(angle)));
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
    public Angle getAngle()
    {
        return cancoderAngle.getValue();
    }

    @Override
    public Angle getTargetAngle()
    {
        return targetAngle;
    }
}