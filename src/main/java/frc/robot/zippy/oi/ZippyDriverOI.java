package frc.robot.zippy.oi;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.zippy.ZippyContainer;

public class ZippyDriverOI implements ZippyOI
{
    private BooleanSupplier isAutoAlignModeCoral;

    /**
     * Process joystick input (meant for XBoxController)
     * 
     * @param input the input to process
     * @return the processed input (squared and deadbanded)
     */
    private static DoubleSupplier processJoystickInput(DoubleSupplier input)
    {
        return () ->
        {
            double x = MathUtil.applyDeadband(input.getAsDouble(), 0.1, 1);
            return -x * Math.abs(x);
        };
    }

    @Override
    public void bindOI(ZippyContainer container)
    {
        CommandXboxController driverJoystick = new CommandXboxController(0);
        isAutoAlignModeCoral = () -> processJoystickInput(driverJoystick::getLeftY).getAsDouble() == 0
                && processJoystickInput(driverJoystick::getLeftX).getAsDouble() == 0
                && processJoystickInput(driverJoystick::getRightX).getAsDouble() == 0;
        container.getDrive()
                .setDefaultCommand(container.getDrive().driveByJoystick(processJoystickInput(driverJoystick::getLeftY),
                        processJoystickInput(driverJoystick::getLeftX),
                        processJoystickInput(driverJoystick::getRightX)));

        driverJoystick.back().onTrue(Commands.runOnce(() -> container.getDrive()
                .resetPose(new Pose2d(container.getDrive().getPose().getTranslation(), FieldConstants.getFieldRotation(
                        DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue))),
                container.getDrive()));

        driverJoystick.start().onTrue(Commands.runOnce(() -> container.getDrive().resetPose(FieldConstants
                .convertPoseByAlliance(FieldConstants.ReefPositions.AB_ALGAE, FieldConstants.getAlliance()))));
        driverJoystick.leftBumper().onTrue(container.getGoToReefPoseCommandLeft().onlyWhile(isAutoAlignModeCoral));
        driverJoystick.rightBumper().onTrue(container.getGoToReefPoseCommandRight().onlyWhile(isAutoAlignModeCoral));
        driverJoystick.y().onTrue(container.driveToCoralStation().onlyWhile(isAutoAlignModeCoral));
        driverJoystick.x().onTrue(container.getGoToProcessorCommand().onlyWhile(isAutoAlignModeCoral));

    }
}
