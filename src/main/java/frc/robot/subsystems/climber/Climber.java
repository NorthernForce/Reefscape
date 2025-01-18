package frc.robot.subsystems.climber;

public class Climber
{
	private ClimberIO io;

	public Climber(ClimberIO climberIO)
	{
		io = climberIO;
	}

	public void climbUpShallow()
	{
		io.climbUpShallow();
	}
}
