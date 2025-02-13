package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.sensor.RollersSensorIO;
import frc.robot.subsystems.rollers.sensor.RollersSensorIOInputsAutoLogged;

/**
 * The rollers subsystem is responsible for controlling the rollers on the
 * robot.
 */

public class Rollers extends SubsystemBase
{
    public final RollersIO m_intakeIO;
    public final RollersSensorIO m_sensorIOAlgae;
    public final RollersSensorIO m_sensorIOCoral;
    private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();
    private final RollersSensorIOInputsAutoLogged m_sensorIOAlgaeInputs = new RollersSensorIOInputsAutoLogged();
    private final RollersSensorIOInputsAutoLogged m_sensorIOCoralInputs = new RollersSensorIOInputsAutoLogged();

    /**
     * Constructs a new Rollers subsystem.
     * 
     * @param intakeIO      The IO for the rollers.
     * @param sensorIOAlgae The IO for the algae sensor.
     * @param sensorIOCoral The IO for the coral sensor.
     */

    public Rollers(RollersIO intakeIO, RollersSensorIO sensorIOAlgae, RollersSensorIO sensorIOCoral)
    {
        m_intakeIO = intakeIO;
        m_sensorIOAlgae = sensorIOAlgae;
        m_sensorIOCoral = sensorIOCoral;
    }

    /**
     * Runs motors to intake piece.
     */

    public void intake(double speed)
    {
        m_intakeIO.set(Math.abs(speed));
    }

    /**
     * Runs motors to outtake piece.
     * 
     * @param speed The speed to outtake at.
     */

    public void outtake(double speed)
    {
        m_intakeIO.set(-Math.abs(speed));
    }

    /**
     * Stops motors.
     */

    public void stop()
    {
        m_intakeIO.set(0);
    }

    /**
     * Returns a command that intakes a piece.
     * 
     * @param speed The speed to intake at.
     * @return The command.
     */

    public Command getIntakeCommand(double speed)
    {
        return run(() -> intake(speed));
    }

    /**
     * Returns a command that outtakes a piece.
     * 
     * @param speed The speed to outtake at.
     * @return The command.
     */

    public Command getOuttakeCommand(double speed)
    {
        return run(() -> outtake(speed));
    }

    /**
     * Returns a command that stops the rollers.
     * 
     * @return The command.
     */

    public Command getStopCommand()
    {
        return run(this::stop);
    }

    /**
     * Updates inputs.
     */

    @Override
    public void periodic()
    {
        m_intakeIO.updateInputs(m_inputs);
        m_sensorIOAlgae.updateInputs(m_sensorIOAlgaeInputs);
        m_sensorIOCoral.updateInputs(m_sensorIOCoralInputs);
        Logger.processInputs(getName() + "/Motor", m_inputs);
        Logger.processInputs(getName() + "/AlgaeSensor", m_sensorIOAlgaeInputs);
        Logger.processInputs(getName() + "/CoralSensor", m_sensorIOCoralInputs);
    }
}
