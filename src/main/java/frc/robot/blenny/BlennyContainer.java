package frc.robot.blenny;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.blenny.constants.BlennyConstants;
import frc.robot.blenny.constants.BlennyTunerConstants;
import frc.robot.blenny.oi.BlennyDriverOI;
import frc.robot.blenny.oi.BlennyProgrammerOI;
import frc.robot.subsystems.phoenix6.PhoenixCommandDrive;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIOTalonFXS;
import frc.robot.subsystems.rollers.sensor.RollersSensorIOUltrasonic;

/**
 * 2025 Competition Robot Container. Name is still a work in progress and will
 * likely change. Blenny is a type of fish. It is also the name of a submarine
 * that was sunk to create an artificial reef.
 */
public class BlennyContainer implements NFRRobotContainer
{
    private final PhoenixCommandDrive drive;
    private final Rollers rollers;

    /**
     * Create a new BlennyContainer
     */
    public BlennyContainer()
    {
        drive = new PhoenixCommandDrive(BlennyTunerConstants.DrivetrainConstants,
                BlennyConstants.DrivetrainConstants.MAX_SPEED, BlennyConstants.DrivetrainConstants.MAX_ANGULAR_SPEED,
                BlennyTunerConstants.FrontLeft, BlennyTunerConstants.FrontRight, BlennyTunerConstants.BackLeft,
                BlennyTunerConstants.BackRight);

        rollers = new Rollers(new RollersIOTalonFXS(30, 31, false), new RollersSensorIOUltrasonic(0, 1, 50),
                new RollersSensorIOUltrasonic(2, 3, 50)); // TODO: FIX THESE IDS
    }

    /**
     * Get the drive subsystem
     * 
     * @return the drive subsystem (PhoenixCommandDrive)
     */
    public PhoenixCommandDrive getDrive()
    {
        return drive;
    }

    /**
     * Get the rollers subsystem from the container
     * 
     * @return the drive subsystem
     */

    public Rollers getRollers()
    {
        return rollers;
    }

    @Override
    public void bindOI()
    {
        switch (Constants.kOI)
        {
        case PROGRAMMER:
            new BlennyProgrammerOI().bindOI(this);
            break;
        case DRIVER:
        default:
            new BlennyDriverOI().bindOI(this);
            break;
        }
    }

    @Override
    public Command getAutonomousCommand()
    {
        return Commands.none();
    }

}
