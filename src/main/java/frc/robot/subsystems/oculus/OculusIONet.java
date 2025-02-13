package frc.robot.subsystems.oculus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class OculusIONet implements OculusIO
{
    NetworkTableInstance nt4Instance;
    NetworkTable nt4Table;
    private IntegerSubscriber questMiso;
    private IntegerPublisher questMosi;

    private DoubleSubscriber questTimestamp;
    private FloatArraySubscriber questPosition;
    private FloatArraySubscriber questQuaternion;
    private FloatArraySubscriber questEulerAngles;
    private DoubleSubscriber questBatteryPercent;

    private float yawOffset = 0.0f;
    private Pose2d resetPosition = new Pose2d();

    public OculusIONet()
    {
        nt4Instance = NetworkTableInstance.getDefault();
        nt4Table = nt4Instance.getTable("questnav");
        questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
        questMosi = nt4Table.getIntegerTopic("mosi").publish();

        questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
        questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]
        { 0.0f, 0.0f, 0.0f });
        questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]
        { 0.0f, 0.0f, 0.0f, 0.0f });
        questEulerAngles = nt4Table.getFloatArrayTopic("eulerangles").subscribe(new float[]
        { 0.0f, 0.0f, 0.0f });
        questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);
    }

    @Override
    public Pose2d getPose()
    {
        return new Pose2d(getQuestNavPose().minus(resetPosition).getTranslation(),
                Rotation2d.fromDegrees(getOculusYaw()));
    }

    @Override
    public double getBatteryPercent()
    {
        return questBatteryPercent.get();
    }

    @Override
    public boolean isConnected()
    {
        return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
    }

    @Override
    public Quaternion getOrientation()
    {
        float[] qqFloats = questQuaternion.get();
        return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
    }

    @Override
    public double timestamp()
    {
        return questTimestamp.get();
    }

    @Override
    public void zeroHeading()
    {
        float[] eulerAngles = questEulerAngles.get();
        yawOffset = eulerAngles[1];
    }

    @Override
    public void zeroPosition()
    {
        resetPosition = getPose();
        if (questMiso.get() != 99)
        {
            questMosi.set(1);
        }
    }

    @Override
    public void cleanUpQuestNavMessages()
    {
        if (questMiso.get() == 99)
        {
            questMosi.set(0);
        }
    }

    @Override
    public float getOculusYaw()
    {
        float[] eulerAngles = questEulerAngles.get();
        var ret = eulerAngles[1] - yawOffset;
        ret %= 360;
        if (ret < 0)
        {
            ret += 360;
        }
        return ret;
    }

    // is this the same for our robot?
    @Override
    public Translation2d getQuestNavTranslation()
    {
        float[] questnavPosition = questPosition.get();
        return new Translation2d(questnavPosition[2], -questnavPosition[0]);
    }

    @Override
    public Pose2d getQuestNavPose()
    {
        var oculousPositionCompensated = getQuestNavTranslation().minus(new Translation2d(0, 0.1651)); // 6.5
        return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
    }
}
