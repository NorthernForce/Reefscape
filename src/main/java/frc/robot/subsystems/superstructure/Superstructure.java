package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ralph.constants.RalphConstants.SuperstructureGoal;
import frc.robot.subsystems.superstructure.elevator.Elevator;

/**
 * Superstructure is a class that controls the superstructure of the robot. The
 * superstructure consists of two elevators, an inner elevator and an outer
 * elevator.
 */
public class Superstructure extends SubsystemBase
{
    public static interface GenericSuperstructureGoal
    {
        public Distance getInnerElevatorGoal();

        public Distance getOuterElevatorGoal();
    }

    public static record SuperstructureState(Distance innerElevatorPosition, Distance outerElevatorPosition) {
    }

    private final Elevator m_innerElevator;
    private final Elevator m_outerElevator;
    private SuperstructureGoal m_goal;
    private final Distance innerElevatorHighPosition;
    private final Distance outerElevatorHighPosition;

    /**
     * Creates a new Superstructure
     * 
     * @param innerElevator the inner elevator
     * @param outerElevator the outer elevator
     */
    public Superstructure(Elevator innerElevator, Elevator outerElevator, Distance innerElevatorHighPosition,
            Distance outerElevatorHighPosition)
    {
        m_innerElevator = innerElevator;
        m_outerElevator = outerElevator;
        this.innerElevatorHighPosition = innerElevatorHighPosition;
        this.outerElevatorHighPosition = outerElevatorHighPosition;
        m_goal = SuperstructureGoal.START;
    }

    public void stop()
    {
        m_innerElevator.stop();
        m_outerElevator.stop();
    }

    public void setGoal(SuperstructureGoal goal)
    {
        m_goal = goal;
    }

    /**
     * Gets the command to move the superstructure to a goal
     * 
     * @param goal the goal to move the superstructure to
     * @return the command to move the superstructure to the goal
     */
    public Command goToGoal(SuperstructureGoal goal)
    {
        return Commands.parallel(m_innerElevator.getMoveToPositionCommand(goal.getInnerElevatorGoal()),
                m_outerElevator.getMoveToPositionCommand(goal.getOuterElevatorGoal()),
                Commands.runOnce(() -> m_goal = goal));
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
        return new SuperstructureState(m_innerElevator.getPosition(), m_outerElevator.getPosition());
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

    public boolean isTooHigh()
    {
        return m_innerElevator.getPosition().gte(innerElevatorHighPosition)
                || m_outerElevator.getPosition().gte(outerElevatorHighPosition);
    }

    public Elevator getInnerElevator()
    {
        return m_innerElevator;
    }

    public Elevator getOuterElevator()
    {
        return m_outerElevator;
    }

    public Command getHomingCommand(double innerElevatorSpeed, double outerElevatorSpeed)
    {
        return Commands.parallel(m_innerElevator.getHomingCommand(innerElevatorSpeed),
                m_outerElevator.getHomingCommand(outerElevatorSpeed));
    }

    public SuperstructureGoal getGoal()
    {
        return m_goal;
    }
}
