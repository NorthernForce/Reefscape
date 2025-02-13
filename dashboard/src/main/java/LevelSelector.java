import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.geom.Ellipse2D;
import java.util.Optional;

import javax.swing.Action;
import javax.swing.BorderFactory;
import javax.swing.JComponent;

/**
 * A component that the possible levels on each side of the hexagon and allows
 * the user to select one of them. The component will notify the action when a
 * level is selected. The action can be set using the setAction method.
 */
public class LevelSelector extends JComponent implements MouseListener
{
    private Optional<Integer> selectedTrapezoid;

    /**
     * The possible levels (L1, R1, L2, R2, L3, R3, L4, R4, ALGAE). The levels are
     * ordered from left to right and from top to bottom. The levels are displayed
     * as rectangles on the left and right sides of the component and as an ellipse
     * at the bottom.
     */
    public static enum Level
    {
        L1, R1, L2, R2, L3, R3, L4, R4, ALGAE
    }

    private final Rectangle[] levelRectangles;
    private final Rectangle[] innerRectangles;
    private final Ellipse2D algaeEllipse;
    private Optional<Level> selectedLevel;
    private Action action = null;
    private boolean[] grayedOut = new boolean[8];

    /**
     * Creates a new LevelSelector.
     */
    public LevelSelector()
    {
        levelRectangles = new Rectangle[8];
        innerRectangles = new Rectangle[8];
        algaeEllipse = new Ellipse2D.Double();
        selectedLevel = Optional.empty();
        selectedTrapezoid = Optional.empty();
        setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        addMouseListener(this);
        fillLevels();
    }

    /**
     * Returns the selected level or an empty optional if no level is selected.
     * 
     * @return the selected level
     */
    public Optional<Level> getSelectedLevel()
    {
        return selectedLevel;
    }

    /**
     * Sets the levels that are grayed out.
     * 
     * @param grayedOut an array of booleans that indicates which levels are grayed
     *                  out
     */
    public void setGrayedOut(boolean[] grayedOut)
    {
        this.grayedOut = grayedOut;
        repaint();
    }

    /**
     * Set the selected trapezoid.
     * 
     * @param trapezoid the selected trapezoid
     */
    public void setSelectedTrapezoid(int trapezoid)
    {
        selectedTrapezoid = Optional.of(trapezoid);
        repaint();
    }

    /**
     * Sets the action that will be notified when a level is selected.
     * 
     * @param action the action that will be notified
     */
    public void setAction(Action action)
    {
        this.action = action;
    }

    /**
     * Fills the levels with rectangles and an ellipse.
     */
    private void fillLevels()
    {
        int centerX = (int) (getWidth() / 2.0);
        int centerY = (int) (getHeight() / 2.0);
        levelRectangles[0] = new Rectangle(0, 10, (int) centerX - 10, (int) centerY / 3 - 10);
        levelRectangles[1] = new Rectangle((int) centerX + 10, 10, (int) centerX - 10, (int) centerY / 3 - 10);
        levelRectangles[2] = new Rectangle(0, (int) centerY / 3 + 10, (int) centerX - 10, (int) centerY / 3 - 10);
        levelRectangles[3] = new Rectangle((int) centerX + 10, (int) centerY / 3 + 10, (int) centerX - 10,
                (int) centerY / 3 - 10);
        levelRectangles[4] = new Rectangle(0, 2 * (int) centerY / 3 + 10, (int) centerX - 10, (int) centerY / 3 - 10);
        levelRectangles[5] = new Rectangle((int) centerX + 10, 2 * (int) centerY / 3 + 10, (int) centerX - 10,
                (int) centerY / 3 - 10);
        levelRectangles[6] = new Rectangle(0, 3 * (int) centerY / 3 + 10, (int) centerX - 10, (int) centerY / 3 - 10);
        levelRectangles[7] = new Rectangle((int) centerX + 10, 3 * (int) centerY / 3 + 10, (int) centerX - 10,
                (int) centerY / 3 - 10);
        int algaeRadius = Math.min(centerX, centerY) / 2 - 10;
        algaeEllipse.setFrame(centerX - algaeRadius, 5 * (int) centerY / 3 - algaeRadius, 2 * algaeRadius,
                2 * algaeRadius);
        for (int i = 0; i < 8; i++)
        {
            innerRectangles[i] = new Rectangle(levelRectangles[i].x + 20, levelRectangles[i].y + 20,
                    levelRectangles[i].width - 40, levelRectangles[i].height - 40);
        }
    }

    @Override
    public void paintComponent(Graphics g)
    {
        super.paintComponent(g);
        fillLevels();
        for (int i = 0; i < 8; i++)
        {
            if (selectedLevel.isPresent() && selectedLevel.get() == Level.values()[i])
            {
                g.setColor(Color.RED);
            } else
            {
                g.setColor(Color.BLUE);
            }
            g.fillRect(levelRectangles[i].x, levelRectangles[i].y, levelRectangles[i].width, levelRectangles[i].height);
            if (grayedOut[i])
            {
                g.setColor(Color.WHITE);
                g.fillRect(innerRectangles[i].x, innerRectangles[i].y, innerRectangles[i].width,
                        innerRectangles[i].height);
            }
            var oldFont = g.getFont();
            g.setFont(oldFont.deriveFont(20.0f));
            g.setColor(Color.WHITE);
            String toDisplay = "";
            if (selectedTrapezoid.isPresent())
            {
                int i2 = (selectedTrapezoid.get() + 2) % 6;
                toDisplay = (char) (i2 * 2 + (i % 2) + 'A') + "";
            }
            toDisplay += 4 - (i / 2);
            int stringWidth = g.getFontMetrics().stringWidth(toDisplay);
            int stringHeight = g.getFontMetrics().getHeight();
            g.drawString(toDisplay, (int) (levelRectangles[i].getCenterX() - stringWidth / 2),
                    (int) (levelRectangles[i].getCenterY() - stringHeight / 2));
            g.setFont(oldFont);
        }
        if (selectedLevel.isPresent() && selectedLevel.get() == Level.ALGAE)
        {
            g.setColor(Color.RED);
        } else
        {
            g.setColor(Color.BLUE);
        }
        g.fillOval((int) algaeEllipse.getX(), (int) algaeEllipse.getY(), (int) algaeEllipse.getWidth(),
                (int) algaeEllipse.getHeight());
    }

    @Override
    public void mouseClicked(MouseEvent e)
    {
        int x = e.getX();
        int y = e.getY();
        for (int i = 0; i < 8; i++)
        {
            if (levelRectangles[i].contains(x, y))
            {
                selectedLevel = Optional.of(Level.values()[i]);
                repaint();
                if (action != null)
                {
                    action.actionPerformed(null);
                }
                return;
            }
        }
        if (algaeEllipse.contains(x, y))
        {
            selectedLevel = Optional.of(Level.ALGAE);
            repaint();
            if (action != null)
            {
                action.actionPerformed(null);
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

    /**
     * Unselects the level that is currently selected.
     */
    public void deselectLevel()
    {
        selectedLevel = Optional.empty();
        repaint();
    }

}
