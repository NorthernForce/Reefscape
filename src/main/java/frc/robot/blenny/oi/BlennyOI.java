package frc.robot.blenny.oi;

import frc.robot.blenny.BlennyContainer;

/**
 * Common interface for all Blenny OI classes
 */
public interface BlennyOI
{
    /**
     * Bind the OI to the BlennyContainer
     * 
     * @param container The BlennyContainer to bind to
     */
    public void bindOI(BlennyContainer container);
}
