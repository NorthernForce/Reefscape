import java.awt.Color;
import java.awt.Graphics;
import java.awt.Rectangle;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import javax.swing.Action;
import javax.swing.JComponent;

public class StationChooser extends JComponent implements MouseListener
{
	private Rectangle rectangle;
	private boolean selected;
	private Action action = null;

	public StationChooser()
	{
		fillStations();
		addMouseListener(this);
		selected = false;
		rectangle = new Rectangle(10, 10, getWidth() - 20, getHeight() - 20);
	}

	public boolean isSelected()
	{
		return selected;
	}

	public void setAction(Action action)
	{
		this.action = action;
	}

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

	public void deselect()
	{
		selected = false;
		repaint();
	}
}
