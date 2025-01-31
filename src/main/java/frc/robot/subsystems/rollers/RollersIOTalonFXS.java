package frc.robot.subsystems.rollers;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

/**
 * The Talon IO for the rollers.
 */

public class RollersIOTalonFXS implements RollersIO
{
    private final TalonFXS intakeMotorOne;
    private final TalonFXS intakeMotorTwo;
    private final StatusSignal<Temperature> motorOneTemperature;
    private final Supplier<Boolean> motorOnePresent;
    private final StatusSignal<Current> motorOneCurrent;
    private final StatusSignal<Temperature> motorTwoTemperature;
    private final Supplier<Boolean> motorTwoPresent;
    private final StatusSignal<Current> motorTwoCurrent;

    /**
     * Constructs a new RollersIOTalonFX.
     * 
     * @param id1      The ID of the first motor.
     * @param id2      The ID of the second motor.
     * @param inverted Whether the mechanism is inverted or not.
     */

    public RollersIOTalonFXS(int id1, int id2, boolean inverted)
    {
        intakeMotorOne = new TalonFXS(id1);
        intakeMotorTwo = new TalonFXS(id2);

        TalonFXSConfiguration configMotorOne = new TalonFXSConfiguration();
        configMotorOne.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        TalonFXSConfiguration configMotorTwo = new TalonFXSConfiguration();
        configMotorTwo.MotorOutput.Inverted = !inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        intakeMotorOne.getConfigurator().refresh(configMotorOne);
        intakeMotorTwo.getConfigurator().refresh(configMotorTwo);

        motorOneTemperature = intakeMotorOne.getDeviceTemp();
        motorOnePresent = () -> intakeMotorOne.isConnected();
        motorOneCurrent = intakeMotorOne.getTorqueCurrent();
        motorTwoTemperature = intakeMotorTwo.getDeviceTemp();
        motorTwoPresent = () -> intakeMotorTwo.isConnected();
        motorTwoCurrent = intakeMotorTwo.getTorqueCurrent();
    }

    /**
     * Sets the speed of the rollers.
     * 
     * @param speed The speed to set the rollers to.
     */

    @Override
    public void set(double speed)
    {
        intakeMotorOne.set(speed);
        intakeMotorTwo.set(speed);
    }

    /**
     * Updates the inputs for the rollers.
     * 
     * @param inputs The inputs to update.
     */

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        inputs.motorOneTemperature = motorOneTemperature.getValue();
        inputs.motorOnePresent = motorOnePresent.get();
        inputs.motorOneCurrent = motorOneCurrent.getValue();
        inputs.motorTwoTemperature = motorTwoTemperature.getValue();
        inputs.motorTwoPresent = motorTwoPresent.get();
        inputs.motorTwoCurrent = motorTwoCurrent.getValue();
    }
}
