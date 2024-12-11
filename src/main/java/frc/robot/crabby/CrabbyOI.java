package frc.robot.crabby;

import frc.robot.robots.CrabbyContainer;

/** A common interface for all OIs, both driver and programmer. */
public interface CrabbyOI
{
	/**
	 * Binds OI to joysticks using container functionality
	 *
	 * @param container the crabby container to use
	 */
	public void bindOI(CrabbyContainer container);
}
