package org.northernforce.util;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A robot container should contain all subsystems and states. Has various
 * utility functions.
 */
public interface NFRRobotContainer
{
	/** Binds the commands from subsystems to the operator interfaces. */
	public void bindOI();

	/** Runs periodically (every 20 ms) regardless of mode. */
	public default void periodic()
	{
	}

	/** Runs periodically (every 20 ms) in only teleop. */
	public default void teleopPeroidic()
	{
	}

	/** Runs periodically (every 20 ms) in only autonomous. */
	public default void autonomousPeriodic()
	{
	}

	/** Runs at the start of autonomous */
	public default void autonomousInit()
	{
	}

	/**
	 * Get the selected autonomous command
	 * 
	 * @return the selected autonomous command
	 */
	public Command getAutonomousCommand();
}
