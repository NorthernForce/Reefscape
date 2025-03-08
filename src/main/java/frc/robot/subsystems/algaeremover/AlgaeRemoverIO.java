package frc.robot.subsystems.algaeremover;

import org.littletonrobotics.junction.AutoLogOutput;

public interface AlgaeRemoverIO {
    public static class AlgaeRemoverIOInputs
    {
        public boolean algaeRemoved = false;
        public boolean armAtTop = true;
    }

    public default void removeAlgae(double speed)
    {
    }

    public default void returnArm(double speed)
    {
    }

    public default void algaeRemoved(boolean algaeRemoved)
    {
    }

    public default void stopMotor()
    {
    }

    public default boolean getAlgaeRemoved()
    {
        return false;
    }

    public default void updateInputs(AlgaeRemoverIOInputs inputs)
    {
    }
}
