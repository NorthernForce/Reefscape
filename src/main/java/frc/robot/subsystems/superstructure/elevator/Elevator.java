package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.superstructure.elevator.brake.BrakeIO;
import frc.robot.subsystems.superstructure.elevator.brake.BrakeIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.elevator.sensor.ElevatorSensorIO;
import frc.robot.subsystems.superstructure.elevator.sensor.ElevatorSensorIOInputsAutoLogged;

/**
 * Elevator is a class to control the elevator.
 */

public class Elevator extends SubsystemBase
{

    private final ElevatorIO m_motor;
    private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();
    private final BrakeIOInputsAutoLogged m_brakeInputs = new BrakeIOInputsAutoLogged();
    private final BrakeIO m_brake;
    private final ElevatorSensorIO m_sensor;
    private final ElevatorSensorIOInputsAutoLogged m_sensorInputs = new ElevatorSensorIOInputsAutoLogged();
    private final Distance m_errorTolerance;
    private Distance targetState;
    private Alert m_motorNotFoundAlert;
    private final SysIdRoutine m_sysIdRoutine;

    /**
     * Creates a new Elevator
     *
     * @param motor  the motor for the elevator
     * @param brake  the brake for the elevator
     * @param sensor the sensor for the elevator
     */
    public Elevator(String name, ElevatorIO motor, BrakeIO brake, ElevatorSensorIO sensor, Distance errorTolerance)
    {
        super(name);
        m_motor = motor;
        m_brake = brake;
        m_sensor = sensor;
        m_errorTolerance = errorTolerance;
        targetState = Meters.of(0);
        m_motorNotFoundAlert = new Alert("Elevator motor not found with name: " + getName(), Alert.AlertType.kWarning);
        m_sysIdRoutine = getSysIdRoutine();
    }

    /**
     * Sets the target position of the elevator
     * 
     * @param position the position to set the elevator to
     */
    public void setTargetPosition(Distance position)
    {
        m_brake.setBrake(false);
        targetState = position;
        m_motor.setTargetPosition(position);
    }

    public void stop()
    {
        m_brake.setBrake(true);
        m_motor.stop();
    }

    public class ElevatorMoveToPositionCommand extends Command
    {
        private Distance position;

        public ElevatorMoveToPositionCommand(Distance position)
        {
            addRequirements(Elevator.this);
            this.position = position;
        }

        @Override
        public void initialize()
        {
            setTargetPosition(position);
        }

        @Override
        public boolean isFinished()
        {
            return isAtTargetPosition();
        }

        @Override
        public void end(boolean interrupted)
        {
            stop();
        }
    }

    /**
     * Gets the command to move the elevator
     * 
     * @param position the position to move the elevator to
     * @return the command to move the elevator
     */
    public Command getMoveToPositionCommand(Distance position)
    {
        return new ElevatorMoveToPositionCommand(position);
    }

    public Command getMoveByJoystick(DoubleSupplier joystick)
    {
        return run(() ->
        {
            m_motor.setSpeed(joystick.getAsDouble(), false);
        });
    }

    public class ElevatorHomingCommand extends Command
    {
        private double speed;

        public ElevatorHomingCommand(double speed)
        {
            addRequirements(Elevator.this);
            this.speed = speed;
        }

        @Override
        public void initialize()
        {
            m_motor.setLowerLimitEnable(false);
        }

        @Override
        public void execute()
        {
            m_motor.setSpeed(-speed, true);
        }

        @Override
        public boolean isFinished()
        {
            return m_sensorInputs.isAtBottom;
        }

        @Override
        public void end(boolean isFinished)
        {
            m_motor.stop();
            m_motor.resetPosition();
            m_motor.setLowerLimitEnable(true);
        }
    }

    public Command getHomingCommand(double homingSpeed)
    {
        return new ElevatorHomingCommand(homingSpeed);
    }

    /**
     * Gets the command to stop the elevator
     * 
     * @return the command to stop the elevator
     */
    public Command getStopCommand()
    {
        return runOnce(() ->
        {
            stop();
        });
    }

    /**
     * sets the inputs for the elevator
     */

    @Override
    public void periodic()
    {
        m_motor.updateInputs(m_inputs);
        m_brake.updateInputs(m_brakeInputs);
        Logger.processInputs(getName() + "/Motor", m_inputs);
        Logger.processInputs(getName() + "/Brake", m_brakeInputs);
        m_sensor.updateInputs(m_sensorInputs);
        Logger.processInputs(getName() + "/Sensor", m_sensorInputs);
        if (m_sensorInputs.isAtBottom)
        {
            m_motor.resetPosition();
        }

        m_motorNotFoundAlert.set(!m_inputs.present);
    }

    /**
     * Gets the position of the elevator
     * 
     * @return the position of the elevator
     */
    public Distance getPosition()
    {
        return m_inputs.position;
    }

    /**
     * Is the elevator at the target position
     * 
     * @return true if the elevator is at the target position, false otherwise
     */
    public boolean isAtTargetPosition()
    {
        return Math.abs(m_inputs.position.in(Meters) - targetState.in(Meters)) <= m_errorTolerance.in(Meters);
    }

    /**
     * Is the elevator at a position
     * 
     * @param position the position to check if the elevator is at
     * @return true if the elevator is at the position, false otherwise
     */
    public boolean isAtPosition(Distance position)
    {
        return Math.abs(m_inputs.position.in(Meters) - position.in(Meters)) <= m_errorTolerance.in(Meters);
    }

    private SysIdRoutine getSysIdRoutine()
    {
        return new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), Seconds.of(4),
                        state -> Logger.recordOutput(getName() + "/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(volts -> m_motor.setVoltage(volts), null, this));
    }

    public Command getSysIdQuasistaicForward()
    {
        return m_sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command getSysIdQuasistaicReverse()
    {
        return m_sysIdRoutine.quasistatic(Direction.kReverse);
    }

    public Command getSysIdDynamicForward()
    {
        return m_sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command getSysIdDynamicReverse()
    {
        return m_sysIdRoutine.dynamic(Direction.kReverse);
    }
}
