package frc.robot.sebastian.oi;

import frc.robot.sebastian.SebastianContainer;

/**
 * Common interface for all Sebastian OI classes
 */
public interface SebastianOI
{
    /**
     * Bind the OI to the SebastianContainer
     * 
     * @param container The SebastianContainer to bind to
     */
    public void bindOI(SebastianContainer container);
}
