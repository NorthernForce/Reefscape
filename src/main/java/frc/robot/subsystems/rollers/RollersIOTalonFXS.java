package frc.robot.subsystems.rollers;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

/**
 * The Talon IO for the rollers.
 */

public class RollersIOTalonFXS implements RollersIO
{
    private final TalonFXS intakeMotorLeft;
    private final TalonFXS intakeMotorRight;
    private final StatusSignal<Temperature> motorLeftTemperature;
    private final Supplier<Boolean> motorLeftPresent;
    private final StatusSignal<Current> motorLeftCurrent;
    private final StatusSignal<Temperature> motorRightTemperature;
    private final Supplier<Boolean> motorRightPresent;
    private final StatusSignal<Current> motorRightCurrent;

    /**
     * Constructs a new RollersIOTalonFX.
     * 
     * @param id1      The ID of the first motor.
     * @param id2      The ID of the second motor.
     * @param inverted Whether the mechanism is inverted or not.
     */

    public RollersIOTalonFXS(int idLeft, int idRight, boolean inverted)
    {
        intakeMotorLeft = new TalonFXS(idLeft);
        intakeMotorRight = new TalonFXS(idRight);

        TalonFXSConfiguration configMotorLeft = new TalonFXSConfiguration();
        configMotorLeft.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        configMotorLeft.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        TalonFXSConfiguration configMotorRight = new TalonFXSConfiguration();
        configMotorRight.MotorOutput.Inverted = !inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        configMotorRight.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        intakeMotorLeft.getConfigurator().apply(configMotorLeft);
        intakeMotorRight.getConfigurator().apply(configMotorRight);

        motorLeftTemperature = intakeMotorLeft.getDeviceTemp();
        motorLeftPresent = () -> intakeMotorLeft.isConnected();
        motorLeftCurrent = intakeMotorLeft.getTorqueCurrent();
        motorRightTemperature = intakeMotorRight.getDeviceTemp();
        motorRightPresent = () -> intakeMotorRight.isConnected();
        motorRightCurrent = intakeMotorRight.getTorqueCurrent();
    }

    /**
     * Sets the speed of the rollers.
     * 
     * @param speed The speed to set the rollers to.
     */

    @Override
    public void set(double speed)
    {
        intakeMotorLeft.set(speed);
        intakeMotorRight.set(speed);
    }

    /**
     * Updates the inputs for the rollers.
     * 
     * @param inputs The inputs to update.
     */

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        inputs.motorLeftTemperature = motorLeftTemperature.getValue();
        inputs.motorLeftPresent = motorLeftPresent.get();
        inputs.motorLeftCurrent = motorLeftCurrent.getValue();
        inputs.motorRightTemperature = motorRightTemperature.getValue();
        inputs.motorRightPresent = motorRightPresent.get();
        inputs.motorRightCurrent = motorRightCurrent.getValue();
    }
}
