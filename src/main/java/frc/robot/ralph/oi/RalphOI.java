package frc.robot.ralph.oi;

import frc.robot.ralph.RalphContainer;

/**
 * Common interface for all Ralph OI classes
 */
public interface RalphOI
{
    /**
     * Bind the OI to the RalphContainer
     * 
     * @param container The RalphContainer to bind to
     */
    public void bindOI(RalphContainer container);
}
