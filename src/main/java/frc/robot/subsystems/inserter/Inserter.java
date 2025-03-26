package frc.robot.subsystems.inserter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ralph.constants.RalphConstants;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;
import frc.robot.subsystems.inserter.sensor.InserterSensorIO;
import frc.robot.subsystems.inserter.sensor.InserterSensorIOInputsAutoLogged;

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

    public void intake(double speed)
    {
        io.set(speed);
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

    public void outtake(double speed)
    {
        io.set(speed);
    }

    public void purge()
    {
        io.set(-RalphConstants.InserterConstants.PURGE_SPEED);
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

    public class CoralIntakeCommand extends Command
    {
        double speed = 0.0;

        public CoralIntakeCommand()
        {
            speed = intakeSpeed;
            addRequirements(Inserter.this);
        }

        public CoralIntakeCommand(double speed)
        {
            this.speed = speed;
            addRequirements(Inserter.this);
        }

        @Override
        public void initialize()
        {
            intake(speed);
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

    public class CoralReintakeCommand extends Command
    {
        double speed;

        public CoralReintakeCommand(double speed)
        {
            this.speed = speed;
            addRequirements(Inserter.this);
        }

        @Override
        public void execute()
        {
            if (!hasCoral())
            {
                io.set(speed);
            } else
            {
                stop();
            }
        }
    }

    public class CoralPurgeCommand extends Command
    {
        public CoralPurgeCommand()
        {
            addRequirements(Inserter.this);
        }

        @Override
        public void initialize()
        {
            purge();
        }

        @Override
        public boolean isFinished()
        {
            return !hasCoral();
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

    public Command intakeCoral()
    {
        return new CoralIntakeCommand();
    }

    public Command intakeCoral(double speed)
    {
        return new CoralIntakeCommand(speed);
    }

    public Command intakeCoralShuffle()
    {
        return Commands.sequence(new CoralPurgeCommand(), new CoralReintakeCommand(0.2));
    }

    public class CoralOuttakeSlowCommand extends Command
    {
        public CoralOuttakeSlowCommand()
        {
            addRequirements(Inserter.this);
        }

        @Override
        public void initialize()
        {
            outtake(RalphConstants.InserterConstants.SLOW_OUTTAKE_SPEED);
        }

        @Override
        public boolean isFinished()
        {
            return !hasCoral();
        }

        @Override
        public void end(boolean interrupted)
        {
            stop();
        }
    }

    public class CoralOuttakeCommand extends Command
    {
        public CoralOuttakeCommand()
        {
            addRequirements(Inserter.this);
        }

        @Override
        public void initialize()
        {
            outtake();
        }

        @Override
        public boolean isFinished()
        {
            return !hasCoral();
        }

        @Override
        public void end(boolean interrupted)
        {
            stop();
        }
    }

    public Command outtakeCoral()
    {
        return new CoralOuttakeCommand();
    }

    public Command outtakeCoralSlow()
    {
        return new CoralOuttakeSlowCommand();
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
