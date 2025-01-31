import java.awt.Color;
import java.awt.Graphics;
import java.awt.Rectangle;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import javax.swing.Action;
import javax.swing.JComponent;

/**
 * A component that displays a station and allows the user to select it. The
 * component will notify the action when the station is selected. The action can
 * be set using the setAction method.
 */
public class StationChooser extends JComponent implements MouseListener
{
    private Rectangle rectangle;
    private boolean selected;
    private Action action = null;

    /**
     * Creates a new StationChooser.
     */
    public StationChooser()
    {
        fillStations();
        addMouseListener(this);
        selected = false;
        rectangle = new Rectangle(10, 10, getWidth() - 20, getHeight() - 20);
    }

    /**
     * Returns whether the station is selected.
     * 
     * @return true if the station is selected, false otherwise
     */
    public boolean isSelected()
    {
        return selected;
    }

    /**
     * Sets the action that will be notified when the station is selected.
     * 
     * @param action
     */
    public void setAction(Action action)
    {
        this.action = action;
    }

    /**
     * Fills the station with a rectangle.
     */
    private void fillStations()
    {
        rectangle = new Rectangle(10, 10, getWidth() - 20, getHeight() - 20);
    }

    @Override
    public void paintComponent(Graphics g)
    {
        super.paintComponent(g);
        fillStations();
        if (selected)
        {
            g.setColor(Color.RED);
        } else
        {
            g.setColor(Color.GREEN);
        }
        g.fillRect(10, 10, getWidth() - 20, getHeight() - 20);
    }

    @Override
    public void mouseClicked(MouseEvent e)
    {
        if (rectangle.contains(e.getPoint()))
        {
            selected = true;
            if (action != null)
            {
                action.actionPerformed(null);
            }
        }
        repaint();
    }

    @Override
    public void mousePressed(MouseEvent e)
    {
    }

    @Override
    public void mouseReleased(MouseEvent e)
    {
    }

    @Override
    public void mouseEntered(MouseEvent e)
    {
    }

    @Override
    public void mouseExited(MouseEvent e)
    {
    }

    /**
     * Deselects the station.
     */
    public void deselect()
    {
        selected = false;
        repaint();
    }
}
