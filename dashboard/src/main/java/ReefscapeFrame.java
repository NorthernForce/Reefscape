import javax.swing.AbstractAction;
import javax.swing.JFrame;
import javax.swing.JPanel;

import java.io.IOException;
import java.util.Arrays;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
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

/**
 * The main class and frame for the Reefscape Display. The frame contains a
 * hexagon selector, a level selector, and three station choosers. The frame
 * also contains a network table that is used to communicate with the robot
 * code.
 */
public class ReefscapeFrame extends JFrame
{
    private final NetworkTable table;
    private final IntegerPublisher choicePublisher;
    private final BooleanArraySubscriber stateSubscriber;
    private final HexagonSelector hexagonSelector;
    private final LevelSelector levelSelector;
    private final StationChooser leftCoral, rightCoral, processor;
    private final JPanel leftPanel, rightPanel;

    /**
     * Creates a new ReefscapeFrame.
     */
    public ReefscapeFrame()
    {
        super("Reef Display");

        NetworkTableInstance.getDefault().setServerTeam(172);
        NetworkTableInstance.getDefault().setServer(new String[]
        { "roborio-172-frc.local", "localhost" });
        NetworkTableInstance.getDefault().startClient4("ReefscapeDisplay");
        table = NetworkTableInstance.getDefault().getTable("ReefscapeDisplay");
        choicePublisher = table.getIntegerTopic("choice").publish();
        boolean state[] = new boolean[48];
        Arrays.fill(state, false);
        stateSubscriber = table.getBooleanArrayTopic("state").subscribe(state);

        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(1280, 800);

        GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
        GraphicsDevice[] graphicsDevices = ge.getScreenDevices();
        if (graphicsDevices.length > 1)
        {
            graphicsDevices[1].setFullScreenWindow(this);
        }

        setLayout(new GridLayout(1, 3));
        hexagonSelector = new HexagonSelector();
        levelSelector = new LevelSelector();
        leftPanel = new JPanel();
        leftPanel.setLayout(new GridLayout(2, 1));
        leftPanel.add(hexagonSelector);
        leftCoral = new StationChooser();
        rightCoral = new StationChooser();
        processor = new StationChooser();
        leftCoral.setAction(new AbstractAction()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                choicePublisher.set(48);
                if (rightCoral.isSelected())
                {
                    rightCoral.deselect();
                }
                if (processor.isSelected())
                {
                    processor.deselect();
                }
                if (levelSelector.getSelectedLevel().isPresent())
                {
                    levelSelector.deselectLevel();
                }
            }
        });
        rightCoral.setAction(new AbstractAction()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                choicePublisher.set(49);
                if (leftCoral.isSelected())
                {
                    leftCoral.deselect();
                }
                if (processor.isSelected())
                {
                    processor.deselect();
                }
                if (levelSelector.getSelectedLevel().isPresent())
                {
                    levelSelector.deselectLevel();
                }
            }
        });
        processor.setAction(new AbstractAction()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                choicePublisher.set(50);
                if (leftCoral.isSelected())
                {
                    leftCoral.deselect();
                }
                if (rightCoral.isSelected())
                {
                    rightCoral.deselect();
                }
                if (levelSelector.getSelectedLevel().isPresent())
                {
                    levelSelector.deselectLevel();
                }
            }
        });
        leftPanel.add(leftCoral);
        hexagonSelector.setAction(new AbstractAction()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                if (hexagonSelector.getSelectedTrapezoid().isPresent() && levelSelector.getSelectedLevel().isPresent())
                {
                    choicePublisher.set(hexagonSelector.getSelectedTrapezoid().get() * 9
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
        add(leftPanel);
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
                }
                if (leftCoral.isSelected())
                {
                    leftCoral.deselect();
                }
                if (rightCoral.isSelected())
                {
                    rightCoral.deselect();
                }
                if (processor.isSelected())
                {
                    processor.deselect();
                }
            }
        });
        rightPanel = new JPanel();
        rightPanel.setLayout(new GridLayout(2, 1));
        rightPanel.add(processor);
        rightPanel.add(rightCoral);
        add(rightPanel);
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
