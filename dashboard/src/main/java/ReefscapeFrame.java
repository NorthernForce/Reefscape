import javax.swing.AbstractAction;
import javax.swing.JFrame;
import java.io.IOException;
import java.util.Arrays;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;

import javax.swing.SwingUtilities;
import org.opencv.core.Core;

import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.jni.EigenJNI;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

public class ReefscapeFrame extends JFrame
{
	private final NetworkTable table;
	private final IntegerPublisher choicePublisher;
	private final BooleanArraySubscriber stateSubscriber;
	private final HexagonSelector hexagonSelector;
	private final LevelSelector levelSelector;
	private final StationChooser stationChooser;

	public ReefscapeFrame()
	{
		super("Reef Display");

		NetworkTableInstance.getDefault().setServerTeam(172);
		NetworkTableInstance.getDefault().startClient4("ReefscapeDisplay");
		table = NetworkTableInstance.getDefault().getTable("ReefscapeDisplay");
		choicePublisher = table.getIntegerTopic("choice").publish();
		boolean state[] = new boolean[48];
		Arrays.fill(state, false);
		stateSubscriber = table.getBooleanArrayTopic("state").subscribe(state);

		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setSize(1280, 800);
		setLayout(new GridLayout(1, 3));
		hexagonSelector = new HexagonSelector();
		levelSelector = new LevelSelector();
		add(hexagonSelector);
		hexagonSelector.setAction(new AbstractAction()
		{
			@Override
			public void actionPerformed(ActionEvent e)
			{
				if (hexagonSelector.getSelectedTrapezoid().isPresent() && levelSelector.getSelectedLevel().isPresent())
				{
					choicePublisher.set(hexagonSelector.getSelectedTrapezoid().get() * 6
							+ levelSelector.getSelectedLevel().get().ordinal());
				}
				if (hexagonSelector.getSelectedTrapezoid().isPresent())
				{
					var m = stateSubscriber.get(state);
					boolean[] grayedOut = new boolean[8];
					for (int i = 0; i < 8; i++)
					{
						grayedOut[i] = m[i + hexagonSelector.getSelectedTrapezoid().get() * 8];
					}
					levelSelector.setGrayedOut(grayedOut);
				}
			}
		});
		add(levelSelector);
		levelSelector.setAction(new AbstractAction()
		{
			@Override
			public void actionPerformed(ActionEvent e)
			{
				if (hexagonSelector.getSelectedTrapezoid().isPresent() && levelSelector.getSelectedLevel().isPresent())
				{
					choicePublisher.set(hexagonSelector.getSelectedTrapezoid().get() * 6
							+ levelSelector.getSelectedLevel().get().ordinal());
					stationChooser.deselect();
				}
			}
		});
		stationChooser = new StationChooser();
		stationChooser.setAction(new AbstractAction()
		{
			@Override
			public void actionPerformed(ActionEvent e)
			{
				if (stationChooser.getSelectedStation().isPresent())
				{
					choicePublisher.set(stationChooser.getSelectedStation().get().ordinal() + 36);
					levelSelector.deselectLevel();
				}
			}
		});
		add(stationChooser);
		setVisible(true);
	}

	public static void main(String[] args) throws IOException
	{
		NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
		WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
		EigenJNI.Helper.setExtractOnStaticLoad(false);
		CameraServerJNI.Helper.setExtractOnStaticLoad(false);
		OpenCvLoader.Helper.setExtractOnStaticLoad(false);

		CombinedRuntimeLoader.loadLibraries(ReefscapeFrame.class, "wpiutiljni", "wpimathjni", "ntcorejni",
				Core.NATIVE_LIBRARY_NAME, "cscorejni");
		SwingUtilities.invokeLater(() ->
		{
			new ReefscapeFrame();
		});
	}
}
