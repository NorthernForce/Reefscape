package frc.robot.subsystems.viewer;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.sebastian.constants.SebastianConstants;
import frc.robot.subsystems.dashboard.DashboardIO;

public class Viewer extends SubsystemBase
{
    private final ViewerIO io;
    private final ViewerIOInputsAutoLogged inputs;
    private final Alert viewerMissingAlert;

    public Viewer(ViewerIO io)
    {
        this.io = io;
        inputs = new ViewerIOInputsAutoLogged();
        viewerMissingAlert = new Alert("Viewer is missing", AlertType.kWarning);
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
        viewerMissingAlert.set(!inputs.connected);
    }

    public Distance getCenterDistance()
    {
        return inputs.centerDistance;
    }

    public Distance getPostDistance()
    {
        return inputs.postDistance;
    }

    public Distance getPostOffset()
    {
        return inputs.postOffset;
    }

    public boolean isPresent()
    {
        return inputs.connected;
    }

    public boolean getPostExist() {
        return inputs.postExist;
    }

    public Command alignWithPost()
    {
        return Commands.defer(() ->
        {
            Viewer viewer = getViewer();
            Distance postOffset = viewer.getPostOffset();

            boolean isRight = postOffset.in(Meters) > 0.0;
            boolean isLeft = postOffset.in(Meters) < 0.0;

            Pose2d currentPose = vision.getLatestPoseEstimate();
            Pose2d newPose;

            if (isRight)
            {
                newPose = new Pose2d(currentPose.getTranslation().plus(new Translation2d(postOffset.in(Meters), 0)),
                        currentPose.getRotation());
            } else if (isLeft)
            {
                newPose = new Pose2d(currentPose.getTranslation().plus(new Translation2d(-postOffset.abs(Meters), 0)),
                        currentPose.getRotation());
            } else
            {
                newPose = currentPose;
            }

            Pose2d backupPose = FieldConstants.getPostBackupPosition(newPose, Feet.of(1));

            return getDrive()
                    .driveToPose(backupPose, SebastianConstants.PathplannerConstants.MAX_VELOCITY,
                            SebastianConstants.PathplannerConstants.MAX_ACCELERATION,
                            SebastianConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                            SebastianConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION)
                    .alongWith(getSuperstructure().getGoToGoalCommand(getDashboard().getSuperstructureGoalForReef()))
                    .andThen(() -> getDrive().driveToPose(newPose, SebastianConstants.PathplannerConstants.MAX_VELOCITY,
                            SebastianConstants.PathplannerConstants.MAX_ACCELERATION,
                            SebastianConstants.PathplannerConstants.MAX_ANGULAR_VELOCITY,
                            SebastianConstants.PathplannerConstants.MAX_ANGULAR_ACCELERATION));
        }, Set.of());
    }
}
