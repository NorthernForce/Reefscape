package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ralph.RalphContainer;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;

public class AlignRobotPost extends Command
{
    private Distance PostOffset;
    private Distance PostDistance;
    private boolean PostExist;

    private double kp = 0.0;
    private double ki = 0.0;
    private double kd = 0.0;

    private final PIDController xPidController;

    private final RalphContainer ralphContainer;

    public AlignRobotPost(RalphContainer ralphContainer)
    {
        this.ralphContainer = ralphContainer;
        xPidController = new PIDController(kp, ki, kd);
    }

    public void XOverideCalc()
    {
        PPHolonomicDriveController.overrideXFeedback(() ->
        {
            double xOffset = PostOffset.in(Meters);
            return xPidController.calculate(xOffset);
        });
    }

    @Override
    public void execute()
    {
        PostOffset = ralphContainer.getViewer().getPostOffset();
        if (!ralphContainer.getViewer().getPostExist() && PostExist)
        {
            PPHolonomicDriveController.clearXFeedbackOverride();
            xPidController.reset();
            PostExist = false;
        } else if (ralphContainer.getViewer().getPostExist() && !PostExist)
        {
            if (PostOffset.in(Meters) >= 0.1)
            {
                XOverideCalc();
            }
            PostExist = true;
        }
    }

    @Override
    public void initialize()
    {
        PPHolonomicDriveController.clearXFeedbackOverride();
    }

    @Override
    public boolean isFinished()
    {
        if (PostOffset.in(Meters) <= 0.1)
        {
            return true;
        } else
        {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        PPHolonomicDriveController.clearXFeedbackOverride();
        xPidController.reset();
    }
}