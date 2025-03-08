package frc.robot.ralph.oi;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.commands.RumbleXBoxController;
import frc.robot.ralph.RalphContainer;
import frc.robot.ralph.constants.RalphConstants;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;

/**
 * Ralph OI for the driver and operator
 */
public class RalphDriverOI implements RalphOI
{
    private static enum TARGET_MODES
    {
        CORAL_STATION, PROCESSOR_STATION, RIGHT_REEF, LEFT_REEF
    }

    private static TARGET_MODES currentMode;
    private static BooleanSupplier isAutoAlignMode;
    private static int autoAlignCounter = 0;
    private static boolean autoCoralMode = false;
    private static Pose2d autoTargetCoralPose = Pose2d.kZero;
    private static boolean leftBumper = false;
    private static boolean rightBumper = false;

    /**
     * Process joystick input (meant for XBoxController)
     * 
     * @param input the input to process
     * @return the processed input (squared and deadbanded)
     */
    private static DoubleSupplier processJoystickInput(DoubleSupplier input, boolean updateCoralMode)
    {

        return () ->
        {
            if (updateCoralMode)
            {
                isAutoAlignMode.getAsBoolean();
                SmartDashboard.putBoolean("Auto Coral Mode", autoCoralMode);
            }
            double x = MathUtil.applyDeadband(input.getAsDouble(), 0.1, 1);
            return -x * Math.abs(x);
        };
    }

    private static void setTargetMode(TARGET_MODES mode, Pose2d targetPose)
    {
        if (mode != TARGET_MODES.LEFT_REEF && mode != TARGET_MODES.RIGHT_REEF)
        {
            autoAlignCounter = 0;
            autoCoralMode = false;
        } else
        {
            if (autoCoralMode)
            {
                if (mode == TARGET_MODES.LEFT_REEF)
                {
                    autoAlignCounter++;
                    leftBumper = true;
                } else
                {
                    rightBumper = true;
                    autoAlignCounter--;
                }
            } else
            {
                autoTargetCoralPose = targetPose;
                SmartDashboard.putNumberArray("Target Pose", new double[]
                { autoTargetCoralPose.getX(), autoTargetCoralPose.getY(),
                        autoTargetCoralPose.getRotation().getRadians() });
                autoCoralMode = true;
                autoAlignCounter = 0;
                if (mode == TARGET_MODES.LEFT_REEF)
                {
                    leftBumper = true;
                } else
                {
                    rightBumper = true;
                }
            }

        }
        currentMode = mode;
        SmartDashboard.putNumber("Auto Align Counter", autoAlignCounter);
        SmartDashboard.putBoolean("Auto Coral Mode", autoCoralMode);
        SmartDashboard.putString("Current Mode", currentMode.toString());
    }

    private static void setTargetMode(TARGET_MODES mode)
    {
        if (mode != TARGET_MODES.LEFT_REEF && mode != TARGET_MODES.RIGHT_REEF)
        {
            autoAlignCounter = 0;
            autoCoralMode = false;
        } else
        {
            if (autoCoralMode)
            {
                if (mode == TARGET_MODES.LEFT_REEF)
                {
                    autoAlignCounter++;
                } else
                {
                    autoAlignCounter--;
                }
            } else
            {
                autoCoralMode = true;
                autoAlignCounter = 0;
            }
        }
        currentMode = mode;
        SmartDashboard.putNumber("Auto Align Counter", autoAlignCounter);
        SmartDashboard.putBoolean("Auto Coral Mode", autoCoralMode);
        SmartDashboard.putString("Current Mode", currentMode.toString());
    }

    static void bindDrive(CommandXboxController driverController, RalphContainer container)
    {
        container.getDrive()
                .setDefaultCommand(container.driveByJoystick(processJoystickInput(driverController::getLeftY, true),
                        processJoystickInput(driverController::getLeftX, true),
                        processJoystickInput(driverController::getRightX, true)));

        driverController.back().onTrue(
                container.getDrive().resetOrientation(FieldConstants.getFieldRotation(FieldConstants.getAlliance())));

        driverController.x()
                .onTrue(Commands.runOnce(() -> setTargetMode(TARGET_MODES.CORAL_STATION))
                        .andThen(container.driveToCoralStation())
                        .onlyWhile(() -> isAutoAlignMode.getAsBoolean() && currentMode == TARGET_MODES.CORAL_STATION));
        driverController.y()
                .onTrue(Commands.runOnce(() -> setTargetMode(TARGET_MODES.PROCESSOR_STATION))
                        .andThen(container.getGoToProcessorCommand()).onlyWhile(
                                () -> isAutoAlignMode.getAsBoolean() && currentMode == TARGET_MODES.PROCESSOR_STATION));

        driverController.leftBumper().onTrue(Commands.runOnce(() ->
        {
            if (!leftBumper)
            {
                setTargetMode(TARGET_MODES.LEFT_REEF, container.getTargetPoseLeft());
            }
        }).andThen(Commands
                .defer(() -> container.driveToPose(container.translatePose(FieldConstants.ReefPositions
                        .getNextRotationalPose(autoTargetCoralPose.getTranslation(), autoAlignCounter))), Set.of())
                .onlyWhile(() -> isAutoAlignMode.getAsBoolean() && currentMode == TARGET_MODES.LEFT_REEF
                        && !driverController.rightBumper().getAsBoolean())));

        driverController.rightBumper().onTrue(Commands.runOnce(() ->
        {
            if (!rightBumper)
            {
                setTargetMode(TARGET_MODES.RIGHT_REEF, container.getTargetPoseRight());
            }
        }).andThen(Commands
                .defer(() -> container.driveToPose(container.translatePose(FieldConstants.ReefPositions
                        .getNextRotationalPose(autoTargetCoralPose.getTranslation(), autoAlignCounter))), Set.of())
                .onlyWhile(() -> isAutoAlignMode.getAsBoolean() && currentMode == TARGET_MODES.RIGHT_REEF
                        && !driverController.leftBumper().getAsBoolean())));

        driverController.leftBumper().whileFalse(Commands.runOnce(() -> leftBumper = false));
        driverController.rightBumper().whileFalse(Commands.runOnce(() -> rightBumper = false));

    }

    static void bindRollers(CommandXboxController driverController, CommandXboxController manipulatorController,
            RalphContainer container)
    {
        manipulatorController.back().onTrue(Commands.runOnce(() -> container.getDashboard().toggleBeamBreak()));

        container.getRollers().setDefaultCommand(container.getRollers().getStopCommand());

        driverController.rightTrigger().whileTrue(container.getOuttakeCommand());

        manipulatorController.rightTrigger().whileTrue(container.getOuttakeCommand());

        manipulatorController.back().whileTrue(container.getOuttakeCommand(1.0));

        container.getRollers().intakeTrigger().onTrue(new RumbleXBoxController(manipulatorController, 0.5, 0.5)
                .alongWith(new RumbleXBoxController(driverController, 0.5, 0.5)));
    }

    static void bindClimber(CommandXboxController driverController, RalphContainer container)
    {
        container.getClimber().setDefaultCommand(container.getClimber().getStopCommand());
        driverController.a().whileTrue(container.getClimber().getClimbExtendCommand());
        driverController.b().whileTrue(container.getClimber().getClimbRetractCommand());
    }

    static void bindSuperstructure(CommandXboxController driverController, CommandXboxController manipulatorController,
            RalphContainer container)
    {
        container.getSuperstructure().getInnerElevator().setDefaultCommand(container.getSuperstructure()
                .getInnerElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getRightY, true)));

        container.getSuperstructure().getOuterElevator().setDefaultCommand(container.getSuperstructure()
                .getOuterElevator().getMoveByJoystick(processJoystickInput(manipulatorController::getLeftY, true)));

        driverController.start()
                .whileTrue(container.getSuperstructure().getHomingCommand(
                        RalphConstants.InnerElevatorConstants.HOMING_SPEED,
                        RalphConstants.OuterElevatorConstants.HOMING_SPEED));

        manipulatorController.start()
                .whileTrue(container.getSuperstructure().getHomingCommand(
                        RalphConstants.InnerElevatorConstants.HOMING_SPEED,
                        RalphConstants.OuterElevatorConstants.HOMING_SPEED));

        manipulatorController.povLeft()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.L1));
        manipulatorController.povUp()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.L2));
        manipulatorController.povRight()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.L3));
        manipulatorController.povDown()
                .whileTrue(container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.L4));
        manipulatorController.a().onTrue(Commands.either(
                container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.CORAL_STATION_PRE)
                        .andThen(container.getSuperstructure().getHomingCommand(0.5, 0.5))
                        .andThen(container.getSuperstructure()
                                .getGoToGoalCommand(RalphConstants.SuperstructureGoal.CORAL_STATION)),
                Commands.none(), () -> container.getSuperstructure().getGoal() != SuperstructureGoal.CORAL_STATION)
                .withTimeout(1.5));
        manipulatorController.b().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.PROCESSOR_STATION));
        manipulatorController.y().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.LOWER_ALGAE));
        manipulatorController.x().whileTrue(
                container.getSuperstructure().getGoToGoalCommand(RalphConstants.SuperstructureGoal.HIGHER_ALGAE));
    }

    @Override
    public void bindOI(RalphContainer container)
    {
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController manipulatorController = new CommandXboxController(1);
        isAutoAlignMode = () ->
        {
            boolean joysticks = processJoystickInput(driverController::getLeftY, false).getAsDouble() == 0
                    && processJoystickInput(driverController::getLeftX, false).getAsDouble() == 0
                    && processJoystickInput(driverController::getRightX, false).getAsDouble() == 0;

            if (!joysticks)
            {
                autoAlignCounter = 0;
                autoCoralMode = false;
            }
            SmartDashboard.putNumber("Auto Align Counter", autoAlignCounter);
            SmartDashboard.putBoolean("Auto Coral Mode", autoCoralMode);

            return joysticks;
        };

        bindDrive(driverController, container);
        bindRollers(driverController, manipulatorController, container);
        bindClimber(driverController, container);
        bindSuperstructure(driverController, manipulatorController, container);

    }
}
