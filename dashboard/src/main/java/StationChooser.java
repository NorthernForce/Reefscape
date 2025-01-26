import java.awt.Color;
import java.awt.Graphics;
import java.awt.Rectangle;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.Optional;

import javax.swing.Action;
import javax.swing.JComponent;

public class StationChooser extends JComponent implements MouseListener
{
	public static enum Station
	{
		LEFT_CORAL, RIGHT_CORAL, PROCESSOR
	}

	private Rectangle leftCoralStation;
	private Rectangle rightCoralStation;
	private Rectangle processor;
	private Optional<Station> selectedStation;
	private Action action = null;

	public StationChooser()
	{
		fillStations();
		addMouseListener(this);
		selectedStation = Optional.empty();
	}

	public Optional<Station> getSelectedStation()
	{
		return selectedStation;
	}

	public void setAction(Action action)
	{
		this.action = action;
	}

	private void fillStations()
	{
		leftCoralStation = new Rectangle(10, 10, getWidth() - 20, getHeight() / 3 - 20);
		rightCoralStation = new Rectangle(10, getHeight() / 3 + 10, getWidth() - 20, getHeight() / 3 - 20);
		processor = new Rectangle(10, 2 * getHeight() / 3 + 10, getWidth() - 20, getHeight() / 3 - 20);
	}

	@Override
	public void paintComponent(Graphics g)
	{
		super.paintComponent(g);
		fillStations();
		if (selectedStation.isPresent() && selectedStation.get() == Station.LEFT_CORAL)
		{
			g.setColor(Color.RED);
			g.fillRect(leftCoralStation.x, leftCoralStation.y, leftCoralStation.width, leftCoralStation.height);
		} else
		{
			g.setColor(Color.GREEN);
			g.fillRect(leftCoralStation.x, leftCoralStation.y, leftCoralStation.width, leftCoralStation.height);
		}
		if (selectedStation.isPresent() && selectedStation.get() == Station.RIGHT_CORAL)
		{
			g.setColor(Color.RED);
			g.fillRect(rightCoralStation.x, rightCoralStation.y, rightCoralStation.width, rightCoralStation.height);
		} else
		{
			g.setColor(Color.GREEN);
			g.fillRect(rightCoralStation.x, rightCoralStation.y, rightCoralStation.width, rightCoralStation.height);
		}
		if (selectedStation.isPresent() && selectedStation.get() == Station.PROCESSOR)
		{
			g.setColor(Color.RED);
			g.fillRect(processor.x, processor.y, processor.width, processor.height);
		} else
		{
			g.setColor(Color.GREEN);
			g.fillRect(processor.x, processor.y, processor.width, processor.height);
		}
		g.setColor(Color.BLACK);
		g.drawString("Left Coral Station", leftCoralStation.x + 50, leftCoralStation.y + 40);
		g.drawString("Right Coral Station", rightCoralStation.x + 50, rightCoralStation.y + 40);
		g.drawString("Processor", processor.x + 50, processor.y + 40);
	}

	@Override
	public void mouseClicked(MouseEvent e)
	{
		if (leftCoralStation.contains(e.getPoint()))
		{
			selectedStation = Optional.of(Station.LEFT_CORAL);
			if (action != null)
			{
				action.actionPerformed(null);
			}
		} else if (rightCoralStation.contains(e.getPoint()))
		{
			selectedStation = Optional.of(Station.RIGHT_CORAL);
			if (action != null)
			{
				action.actionPerformed(null);
			}
		} else if (processor.contains(e.getPoint()))
		{
			selectedStation = Optional.of(Station.PROCESSOR);
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
		selectedStation = Optional.empty();
		repaint();
	}
}
