package frc.robot.subsystems.elevator;

import frc.robot.zippy.constants.ZippyConstants;

public enum ElevatorState
{
	L1(1, ZippyConstants.FieldConstants.L1_HEIGHT), L2(2, ZippyConstants.FieldConstants.L2_HEIGHT),
	L3(3, ZippyConstants.FieldConstants.L3_HEIGHT), L4(4, ZippyConstants.FieldConstants.L4_HEIGHT);

	private double level;
    private double height;

	ElevatorState(int level, double height)
	{
		this.level = level;
        this.height = height;
	}

	public double getLevel()
	{
		return level;
	}

    public double getHeight()
    {
        return height;
    }
}
