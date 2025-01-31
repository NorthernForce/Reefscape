package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.brake.BrakeIO;
import frc.robot.subsystems.elevator.brake.BrakeIOInputsAutoLogged;
import frc.robot.subsystems.elevator.sensor.ElevatorSensorIO;
import frc.robot.subsystems.elevator.sensor.ElevatorSensorIOInputsAutoLogged;
import frc.robot.zippy.constants.ZippyConstants.ElevatorConstants.ElevatorState;

/**
 * Elevator is a class to control the elevator.
 */

public class Elevator extends SubsystemBase
{

    public static interface GenericElevatorGoal
    {
        public Distance getOuterHeight();

        public Distance getInnerHeight();
    }

    private ElevatorIO m_motorOuter;
    private ElevatorIO m_motorInner;
    private final ElevatorIOInputsAutoLogged m_inputsOuter = new ElevatorIOInputsAutoLogged();
    private final ElevatorIOInputsAutoLogged m_inputsInner = new ElevatorIOInputsAutoLogged();
    private final BrakeIOInputsAutoLogged m_brakeInputsOuter = new BrakeIOInputsAutoLogged();
    private final BrakeIOInputsAutoLogged m_brakeInputsInner = new BrakeIOInputsAutoLogged();
    private BrakeIO m_brakeOuter;
    private BrakeIO m_brakeInner;
    private final ElevatorSensorIO m_sensorOuter;
    private final ElevatorSensorIO m_sensorInner;

    private final ElevatorSensorIOInputsAutoLogged m_sensorInputsOuter = new ElevatorSensorIOInputsAutoLogged();
    private final ElevatorSensorIOInputsAutoLogged m_sensorInputsInner = new ElevatorSensorIOInputsAutoLogged();

    /**
     * Creates a new Elevator
     * 
     * @param ioOuter    the outer elevator io
     * @param ioInner    the inner elevator io
     * @param breakOuter the outer brake io
     * @param breakInner the inner brake io
     */

    public Elevator(ElevatorIO ioOuter, ElevatorIO ioInner, BrakeIO breakOuter, BrakeIO breakInner,
            ElevatorSensorIO sensorOuter, ElevatorSensorIO sensorInner)
    {
        m_motorOuter = ioOuter;
        m_motorInner = ioInner;
        m_brakeOuter = breakOuter;
        m_brakeInner = breakInner;
        m_sensorOuter = sensorOuter;
        m_sensorInner = sensorInner;
    }

    /**
     * Gets the outer elevator command
     * 
     * @param level the level to move the outer elevator to
     * @return the command to move the outer elevator
     */

    public Command getLevelOuterCommand(ElevatorState level)
    {
        return new Command()
        {
            @Override
            public void initialize()
            {
                m_brakeOuter.setBreak(false);
                m_motorOuter.setTargetPosition(level.getOuterHeight());
            }

            @Override
            public boolean isFinished()
            {
                return m_inputsOuter.isAtTargetPosition;
            }

            @Override
            public void end(boolean isInterrupted)
            {
                m_motorOuter.stop();
                m_brakeOuter.setBreak(true);
            }
        };
    }

    /**
     * Gets the inner elevator command
     * 
     * @param level the level to go to
     * @return the command to move the inner elevator
     */

    public Command getLevelInnerCommand(ElevatorState level)
    {
        return new Command()
        {
            @Override
            public void initialize()
            {
                m_brakeInner.setBreak(false);
                m_motorInner.setTargetPosition(level.getInnerHeight());
            }

            @Override
            public boolean isFinished()
            {
                return m_inputsInner.isAtTargetPosition;
            }

            @Override
            public void end(boolean isInterrupted)
            {
                m_motorInner.stop();
                m_brakeInner.setBreak(true);
            }
        };
    }

    /**
     * Gets the command to move the inner and outer elevators to a level
     * 
     * @param level the level to move the elevator to
     * @return the command to move the elevator to the level
     */

    public ParallelCommandGroup getLevelCommand(ElevatorState level)
    {
        return new ParallelCommandGroup(getLevelOuterCommand(level), getLevelInnerCommand(level));
    }

    /**
     * Gets the command to move the elevator to L1
     * 
     * @return the command to move the elevator to L1
     */

    public Command getL1Command()
    {
        return getLevelCommand(ElevatorState.L1);
    }

    /**
     * Gets the command to move the elevator to L2
     * 
     * @return the command to move the elevator to L2
     */

    public Command getL2Command()
    {
        return getLevelCommand(ElevatorState.L2);
    }

    /**
     * Gets the command to move the elevator to L3
     * 
     * @return the command to move the elevator to L3
     */

    public Command getL3Command()
    {
        return getLevelCommand(ElevatorState.L3);
    }

    /**
     * Gets the command to move the elevator to L4
     * 
     * @return the command to move the elevator to L4
     */

    public Command getL4Command()
    {
        return getLevelCommand(ElevatorState.L4);
    }

    /**
     * sets the inputs for the elevator
     */

    @Override
    public void periodic()
    {
        m_motorOuter.updateInputs(m_inputsOuter);
        m_motorInner.updateInputs(m_inputsInner);
        m_brakeOuter.updateInputs(m_brakeInputsOuter);
        m_brakeInner.updateInputs(m_brakeInputsInner);
        Logger.processInputs(getName() + "/Outer", m_inputsOuter);
        Logger.processInputs(getName() + "/Inner", m_inputsInner);
        Logger.processInputs(getName() + "/BrakeOuter", m_brakeInputsOuter);
        Logger.processInputs(getName() + "/BrakeInner", m_brakeInputsInner);
        m_sensorOuter.updateInputs(m_sensorInputsOuter);
        m_sensorInner.updateInputs(m_sensorInputsInner);
        Logger.processInputs(getName() + "/SensorOuter", m_sensorInputsOuter);
        Logger.processInputs(getName() + "/SensorInner", m_sensorInputsInner);
        if (m_sensorInputsOuter.isAtBottom)
        {
            m_motorOuter.resetPosition();
        }
        if (m_sensorInputsInner.isAtBottom)
        {
            m_motorInner.resetPosition();
        }
    }
}
