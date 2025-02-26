package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
    private final Alert m_intakeLeftMotorMissing = new Alert("Intake left motor is missing", AlertType.kError);
    private final Alert m_intakeRightMotorMissing = new Alert("Intake right motor is missing", AlertType.kError);
    private final double intakeSpeed;
    private final double outtakeSpeed;

    /**
     * Constructs a new Rollers subsystem.
     * 
     * @param intakeIO      The IO for the rollers.
     * @param sensorIOAlgae The IO for the algae sensor.
     * @param sensorIOCoral The IO for the coral sensor.
     */

    public Rollers(RollersIO intakeIO, RollersSensorIO sensorIOAlgae, RollersSensorIO sensorIOCoral, double intakeSpeed,
            double outtakeSpeed)
    {
        m_intakeIO = intakeIO;
        m_sensorIOAlgae = sensorIOAlgae;
        m_sensorIOCoral = sensorIOCoral;
        this.intakeSpeed = intakeSpeed;
        this.outtakeSpeed = outtakeSpeed;
    }

    /**
     * Runs motors to intake piece.
     */

    public void intake()
    {
        m_intakeIO.set(intakeSpeed);
    }

    /**
     * Runs motors to outtake piece.
     * 
     * @param speed The speed to outtake at.
     */

    public void outtake()
    {
        m_intakeIO.set(-outtakeSpeed);
    }

    /**
     * Stops motors.
     */

    public void stop()
    {
        m_intakeIO.set(0);
    }

    @AutoLogOutput
    public boolean hasAlgae()
    {
        return m_sensorIOAlgaeInputs.hasPiece;
    }

    @AutoLogOutput
    public boolean hasCoral()
    {
        return m_sensorIOCoralInputs.hasPiece;
    }

    public class CoralIntakeCommand extends Command
    {
        public CoralIntakeCommand()
        {
            addRequirements(Rollers.this);
        }

        @Override
        public void initialize()
        {
            intake();
        }

        @Override
        public boolean isFinished()
        {
            return hasCoral();
        }

        @Override
        public void end(boolean interrupted)
        {
            stop();
        }
    }

    /**
     * Returns a command that intakes a coral.
     * 
     * @param speed The speed to intake at.
     * @return The command.
     */

    public Command getCoralIntakeCommand()
    {
        return new CoralIntakeCommand();
    }

    /**
     * Returns a command that intakes an algae.
     * 
     * @param speed The speed to intake at.
     * @return The command.
     */

    public Command getAlgaeIntakeCommand()
    {
        return run(() -> intake()).until(() -> hasAlgae());
    }

    /**
     * Returns a command that outtakes a piece. Does not stop
     * 
     * @param speed The speed to outtake at.
     * @return The command.
     */

    public Command getOuttakeCommand()
    {
        return run(() -> outtake());
    }

    public Command getOuttakeUntilEmptyCommand()
    {
        return run(() -> outtake()).until(() -> !hasAlgae() && !hasCoral());
    }

    public Command getOuttakeCoralCommand()
    {
        return run(() -> outtake()).until(() -> !hasCoral());
    }

    public Command getOuttakeAlgaeCommand()
    {
        return run(() -> outtake()).until(() -> !hasAlgae());
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

    public Command getHoldAlgae()
    {
        return run(() -> intake());
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
        m_intakeLeftMotorMissing.set(!m_inputs.motorLeftPresent);
        m_intakeRightMotorMissing.set(!m_inputs.motorRightPresent);
    }
}
