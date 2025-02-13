import javax.swing.Action;
import javax.swing.ImageIcon;
import javax.swing.JComponent;

import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.Optional;

/**
 * A component that displays a hexagon and allows the user to select one of the
 * trapezoids that make up the hexagon. The component will notify the action
 * when a trapezoid is selected. The action can be set using the setAction
 * method.
 */
public class HexagonSelector extends JComponent implements MouseListener
{
    private final Polygon[] trapezoids;
    private Optional<Integer> selectedTrapezoid;
    private final Point[] points;
    private Action action = null;
    private boolean connected = false;
    private final ImageIcon connectedIcon;
    private final ImageIcon disconnectedIcon;

    /**
     * Creates a new HexagonSelector.
     */
    public HexagonSelector()
    {
        var largeDisconnectedIcon = new ImageIcon(getClass().getResource("/disconnected.png"));
        disconnectedIcon = new ImageIcon(largeDisconnectedIcon.getImage().getScaledInstance(50, 50, 0));
        var largeConnectedIcon = new ImageIcon(getClass().getResource("/connected.png"));
        connectedIcon = new ImageIcon(largeConnectedIcon.getImage().getScaledInstance(50, 50, 0));
        trapezoids = new Polygon[6];
        points = new Point[6];
        selectedTrapezoid = Optional.empty();
        addMouseListener(this);
    }

    /**
     * Sets the action that will be notified when a trapezoid is selected.
     * 
     * @param action
     */
    public void setAction(Action action)
    {
        this.action = action;
    }

    /**
     * Sets whether the display is connected or not.
     * 
     * @param connected whether the display is connected
     */
    public void setConnected(boolean connected)
    {
        this.connected = connected;
        repaint();
    }

    /**
     * Fills the hexagon with trapezoids.
     */
    private void fillHexagons()
    {
        double centerX = getWidth() / 2.0;
        double centerY = getHeight() / 2.0;
        double radius = Math.min(getWidth(), getHeight()) / 2.0;
        Point[] points = new Point[6];
        Point[] innerPoints = new Point[6];
        for (int i = 0; i < 6; i++)
        {
            double angle = Math.PI * 2 - Math.PI / 3.0 * i;
            double x = centerX + radius * Math.cos(angle);
            double y = centerY + radius * Math.sin(angle);
            points[i] = new Point((int) x, (int) y);
            double innerRadius = radius / 2.0;
            x = centerX + innerRadius * Math.cos(angle);
            y = centerY + innerRadius * Math.sin(angle);
            innerPoints[i] = new Point((int) x, (int) y);
            double averageRadius = (radius + innerRadius) / 2.0;
            angle -= Math.PI / 6.0;
            x = centerX + averageRadius * Math.cos(angle);
            y = centerY + averageRadius * Math.sin(angle);
            this.points[i] = new Point((int) x, (int) y);
        }
        for (int i = 0; i < 6; i++)
        {
            int next = (i + 1) % 6;
            int[] xPoints =
            { points[i].x, points[next].x, innerPoints[next].x, innerPoints[i].x };
            int[] yPoints =
            { points[i].y, points[next].y, innerPoints[next].y, innerPoints[i].y };
            trapezoids[i] = new Polygon(xPoints, yPoints, 4);
        }
    }

    @Override
    protected void paintComponent(Graphics g)
    {
        var g2d = (Graphics2D) g;
        super.paintComponent(g);
        fillHexagons();
        var originalFont = g.getFont();
        g.setFont(g.getFont().deriveFont(20.0f));
        for (int i = 0; i < 6; i++)
        {
            if (selectedTrapezoid.isPresent() && selectedTrapezoid.get() == i)
            {
                g.setColor(Color.RED);
            } else
            {
                g.setColor(Color.BLACK);
            }
            g.fillPolygon(trapezoids[i]);
            int i2 = (i + 2) % 6;
            String toDisplay = String.format("%c        %c", (char) ('A' + i2 * 2), (char) ('A' + i2 * 2 + 1));
            var originalTransform = ((Graphics2D) g).getTransform();
            var fontMetrics = g.getFontMetrics();
            var textHeight = fontMetrics.getHeight();
            var textWidth = fontMetrics.stringWidth(toDisplay);
            g.translate(this.points[i].x, this.points[i].y);
            g2d.rotate(-i * Math.PI / 3.0 + Math.PI / 6.0 - Math.PI / 2.0 - Math.PI / 3.0);
            g.setColor(Color.WHITE);
            g.drawString(toDisplay, -textWidth / 2, -textHeight / 2);
            g2d.setTransform(originalTransform);
        }
        if (connected)
        {
            connectedIcon.paintIcon(this, g, 0, 0);
        } else
        {
            disconnectedIcon.paintIcon(this, g, 0, 0);
        }
        g.setFont(originalFont);
    }

    /**
     * Returns the index of the selected trapezoid or an empty optional if no
     * trapezoid is selected.
     * 
     * @return the index of the selected trapezoid
     */
    public Optional<Integer> getSelectedTrapezoid()
    {
        return selectedTrapezoid;
    }

    /**
     * Unselects the trapezoid that is currently selected.
     */
    public void unselectTrapezoid()
    {
        selectedTrapezoid = Optional.empty();
        repaint();
    }

    @Override
    public void mouseClicked(MouseEvent e)
    {
        for (int i = 0; i < 6; i++)
        {
            if (trapezoids[i].contains(e.getPoint()))
            {
                selectedTrapezoid = Optional.of(i);
                repaint();
                if (action != null)
                {
                    action.actionPerformed(null);
                }
                break;
            }
        }
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
}
