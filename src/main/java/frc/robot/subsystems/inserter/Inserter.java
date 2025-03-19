package frc.robot.subsystems.inserter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.inserter.sensor.InserterSensorIO;
import frc.robot.subsystems.inserter.sensor.InserterSensorIOInputsAutoLogged;
import frc.robot.subsystems.inserter.commands.CoralOuttakeCommand;
import frc.robot.subsystems.inserter.commands.CoralIntakeCommand;

/**
 * The inserter subsystem is responsible for controlling the rollers for
 * inserting on the robot.
 */

public class Inserter extends SubsystemBase
{
    private final InserterIO io;
    private final InserterSensorIO sensorIO;
    private final InserterIOInputsAutoLogged inputs = new InserterIOInputsAutoLogged();
    private final InserterSensorIOInputsAutoLogged sensorInputs = new InserterSensorIOInputsAutoLogged();
    private final Alert motorMissingAlert = new Alert("Intake left motor is missing", AlertType.kError);
    private final double intakeSpeed;
    private final double outtakeSpeed;

    /**
     * Constructs a new Inserter subsystem.
     * 
     * @param intakeIO      The IO for the inserter.
     * @param sensorIOAlgae The IO for the algae sensor.
     * @param sensorIOCoral The IO for the coral sensor.
     */

    public Inserter(InserterIO io, InserterSensorIO sensorIO, double intakeSpeed, double outtakeSpeed)
    {
        this.io = io;
        this.sensorIO = sensorIO;
        this.intakeSpeed = intakeSpeed;
        this.outtakeSpeed = outtakeSpeed;
    }

    public Trigger intakeTrigger()
    {
        return new Trigger(this::hasCoral);
    }

    public Trigger readyToIntakeTrigger()
    {
        return new Trigger(this::readyToIntake);
    }

    /**
     * Runs motors to intake piece.
     */

    public void intake()
    {
        io.set(intakeSpeed);
    }

    /**
     * Runs motors to outtake piece.
     * 
     * @param speed The speed to outtake at.
     */

    public void outtake()
    {
        io.set(outtakeSpeed);
    }

    /**
     * Stops motors.
     */

    public void stop()
    {
        io.set(0);
    }

    @AutoLogOutput
    public boolean hasCoral()
    {
        return sensorInputs.hasPiece;
    }

    @AutoLogOutput
    public boolean readyToIntake()
    {
        return !sensorInputs.hasPiece;
    }

    /**
     * Returns a command that intakes a coral.
     * 
     * @param speed The speed to intake at.
     * @return The command.
     */

    public Command intakeCoral()
    {
        return new CoralIntakeCommand(Inserter.this);
    }

    public Command outtakeCoral()
    {
        return new CoralOuttakeCommand(Inserter.this);
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
        io.updateInputs(inputs);
        sensorIO.updateInputs(sensorInputs);
        Logger.processInputs(getName() + "/Motor", inputs);
        Logger.processInputs(getName() + "/Sensor", sensorInputs);
        motorMissingAlert.set(!inputs.motorPresent);
    }
}
