import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.geom.Ellipse2D;
import java.util.Optional;

import javax.swing.Action;
import javax.swing.BorderFactory;
import javax.swing.JComponent;

public class LevelSelector extends JComponent implements MouseListener
{

	public static enum Level
	{
		L1, R1, L2, R2, L3, R3, L4, R4, ALGAE
	}

	private final Rectangle[] levelRectangles;
	private final Ellipse2D algaeEllipse;
	private Optional<Level> selectedLevel;
	private Action action = null;
	private boolean[] grayedOut = new boolean[8];

	public LevelSelector()
	{
		levelRectangles = new Rectangle[8];
		algaeEllipse = new Ellipse2D.Double();
		selectedLevel = Optional.empty();
		setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
		addMouseListener(this);
		fillLevels();
	}

	public Optional<Level> getSelectedLevel()
	{
		return selectedLevel;
	}

	public void setGrayedOut(boolean[] grayedOut)
	{
		this.grayedOut = grayedOut;
		repaint();
	}

	public void setAction(Action action)
	{
		this.action = action;
	}

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
				g.setColor(grayedOut[i] ? Color.GRAY : Color.RED);
			} else
			{
				g.setColor(Color.BLUE);
			}
			g.fillRect(levelRectangles[i].x, levelRectangles[i].y, levelRectangles[i].width, levelRectangles[i].height);
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

	public void deselectLevel()
	{
		selectedLevel = Optional.empty();
		repaint();
	}

}
