package frc.robot.subsystems.elevator;

import frc.robot.zippy.constants.ZippyConstants;

public enum ElevatorState
{
	L1(ZippyConstants.FieldConstants.L1_HEIGHT), L2(ZippyConstants.FieldConstants.L2_HEIGHT),
	L3(ZippyConstants.FieldConstants.L3_HEIGHT), L4(ZippyConstants.FieldConstants.L4_HEIGHT);

	private double level;

	ElevatorState(double level)
	{
		this.level = level;
	}

	public double getLevel()
	{
		return level;
	}
}
