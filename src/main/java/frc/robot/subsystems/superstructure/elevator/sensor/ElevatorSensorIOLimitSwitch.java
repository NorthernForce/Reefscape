package frc.robot.subsystems.superstructure.elevator.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSensorIOLimitSwitch implements ElevatorSensorIO
{
    private final DigitalInput limitSwitch;

    /**
     * Creates a new ElevatorSensorIOLimitSwitch
     * 
     * @param channel the DIO channel on the RoboRIO
     */
    public ElevatorSensorIOLimitSwitch(int channel)
    {
        limitSwitch = new DigitalInput(channel);
    }

    /**
     * Updates the inputs for the elevator
     * 
     * @param inputs the inputs to update
     */
    @Override
    public void updateInputs(ElevatorSensorIOInputs inputs)
    {
        inputs.isAtBottom = !limitSwitch.get();
    }

}
