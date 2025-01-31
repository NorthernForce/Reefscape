package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;

/**
 * Superstructure is a class that controls the superstructure of the robot. The
 * superstructure consists of two elevators, an inner elevator and an outer
 * elevator, and a wrist.
 */
public class Superstructure extends SubsystemBase
{
    public static interface GenericSuperstructureGoal
    {
        public Distance getInnerElevatorGoal();

        public Distance getOuterElevatorGoal();

        public Angle getWristGoal();
    }

    public static record SuperstructureState(Distance innerElevatorPosition, Distance outerElevatorPosition,
            Angle wristPosition) {
    }

    private final Elevator m_innerElevator;
    private final Elevator m_outerElevator;

    /**
     * Creates a new Superstructure
     * 
     * @param innerElevator the inner elevator
     * @param outerElevator the outer elevator
     */
    public Superstructure(Elevator innerElevator, Elevator outerElevator)
    {
        m_innerElevator = innerElevator;
        m_outerElevator = outerElevator;
    }

    /**
     * Gets the command to move the superstructure to a goal
     * 
     * @param goal the goal to move the superstructure to
     * @return the command to move the superstructure to the goal
     */
    public Command getGoToGoalCommand(GenericSuperstructureGoal goal)
    {
        return Commands.parallel(m_innerElevator.getMoveToPositionCommand(goal.getInnerElevatorGoal()),
                m_outerElevator.getMoveToPositionCommand(goal.getOuterElevatorGoal()));
    }

    public Command getStopCommand()
    {
        return Commands.parallel(m_innerElevator.getStopCommand(), m_outerElevator.getStopCommand());
    }

    /**
     * Gets the state of the superstructure
     * 
     * @return the state of the superstructure
     */
    @AutoLogOutput
    public SuperstructureState getState()
    {
        return new SuperstructureState(m_innerElevator.getPosition(), m_outerElevator.getPosition(), Rotations.of(0));
    }

    /**
     * Checks if the superstructure is at the goal position
     * 
     * @return true if the superstructure is at the goal position, false otherwise
     */
    @AutoLogOutput
    public boolean isAtGoal()
    {
        return m_innerElevator.isAtTargetPosition() && m_outerElevator.isAtTargetPosition();
    }

    /**
     * Checks if the superstructure is at a goal
     * 
     * @param goal the goal to check
     * @return true if the superstructure is at the goal, false otherwise
     */
    public boolean isAtGoal(GenericSuperstructureGoal goal)
    {
        return m_innerElevator.isAtPosition(goal.getInnerElevatorGoal())
                && m_outerElevator.isAtPosition(goal.getOuterElevatorGoal());
    }
}
