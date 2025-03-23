package frc.robot.subsystems.inserter;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * The TalonFXS IO for the inserter.
 */

public class InserterIOTalonFXS implements InserterIO
{
    private final TalonFXS motor;
    private final StatusSignal<Temperature> motorTemperature;
    private final Supplier<Boolean> motorPresent;
    private final StatusSignal<Current> motorCurrent;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Angle> motorPosition;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0).withEnableFOC(true);

    /**
     * Constructs a new InserterIOTalonFXS.
     * 
     * @param id1      The ID of the first motor.
     * @param id2      The ID of the second motor.
     * @param inverted Whether the mechanism is inverted or not.
     */

    public InserterIOTalonFXS(int id, boolean inverted)
    {
        motor = new TalonFXS(id);

        TalonFXSConfiguration configMotor = new TalonFXSConfiguration();
        configMotor.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        configMotor.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configMotor.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        motor.getConfigurator().apply(configMotor);

        motorTemperature = motor.getDeviceTemp();
        motorPresent = () -> motor.isConnected() && !motor.getFault_HallSensorMissing().getValue();
        motorCurrent = motor.getTorqueCurrent();
        motorVoltage = motor.getMotorVoltage();
        motorVelocity = motor.getVelocity();
        motorPosition = motor.getPosition();
    }

    /**
     * Sets the speed of the inserter.
     * 
     * @param speed The speed to set the inserter to.
     */

    @Override
    public void set(double speed)
    {
        motor.setControl(dutyCycle.withOutput(speed));
    }

    /**
     * Updates the inputs for the inserter.
     * 
     * @param inputs The inputs to update.
     */

    @Override
    public void updateInputs(InserterIOInputs inputs)
    {
        BaseStatusSignal.refreshAll(motorTemperature, motorCurrent, motorVoltage, motorVelocity, motorPosition);
        inputs.motorTemperature = motorTemperature.getValue();
        inputs.motorPresent = motorPresent.get();
        inputs.motorCurrent = motorCurrent.getValue();
        inputs.motorVoltage = motorVoltage.getValue();
        inputs.motorVelocity = motorVelocity.getValue();
        inputs.motorPosition = motorPosition.getValue();
    }
}
