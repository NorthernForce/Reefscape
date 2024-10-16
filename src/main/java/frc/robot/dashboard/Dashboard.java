package frc.robot.dashboard;

public abstract class Dashboard {
    public testSend() {
        Shuffleboard.getTab("Numbers").add("Pi", 3.14)
    }
}