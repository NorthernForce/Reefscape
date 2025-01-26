import javax.swing.Action;
import javax.swing.JComponent;

import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.Optional;

public class HexagonSelector extends JComponent implements MouseListener
{
	private final Polygon[] trapezoids;
	private Optional<Integer> selectedTrapezoid;
	private Action action = null;

	public HexagonSelector()
	{
		trapezoids = new Polygon[6];
		selectedTrapezoid = Optional.empty();
	}

	public void setAction(Action action)
	{
		this.action = action;
	}

	private void fillHexagons()
	{
		double centerX = getWidth() / 2.0;
		double centerY = getHeight() / 2.0;
		double radius = Math.min(getWidth(), getHeight()) / 2.0;
		Point[] points = new Point[6];
		Point[] innerPoints = new Point[6];
		for (int i = 0; i < 6; i++)
		{
			double angle = Math.PI / 3.0 * i;
			double x = centerX + radius * Math.cos(angle);
			double y = centerY + radius * Math.sin(angle);
			points[i] = new Point((int) x, (int) y);
			double innerRadius = radius / 2.0;
			x = centerX + innerRadius * Math.cos(angle);
			y = centerY + innerRadius * Math.sin(angle);
			innerPoints[i] = new Point((int) x, (int) y);
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
		addMouseListener(this);
	}

	@Override
	protected void paintComponent(Graphics g)
	{
		super.paintComponent(g);
		fillHexagons();
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
		}
	}

	@Override
	public Dimension getPreferredSize()
	{
		return new Dimension(800, 800);
	}

	public Optional<Integer> getSelectedTrapezoid()
	{
		return selectedTrapezoid;
	}

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
