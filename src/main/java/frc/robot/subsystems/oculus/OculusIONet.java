package frc.robot.subsystems.oculus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class OculusIONet implements OculusIO
{
	NetworkTableInstance nt4Instance;
	NetworkTable nt4Table;
	private IntegerSubscriber questMiso;
	private IntegerPublisher questMosi;

	// Subscribe to the Network Tables oculus data topics
	private IntegerSubscriber questFrameCount;
	private DoubleSubscriber questTimestamp;
	private FloatArraySubscriber questPosition;
	private FloatArraySubscriber questQuaternion;
	private FloatArraySubscriber questEulerAngles;
	private DoubleSubscriber questBattery;

	public OculusIONet(String tableName)
	{
		nt4Instance = NetworkTableInstance.getDefault();
		nt4Table = nt4Instance.getTable(tableName);
		questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
		questMosi = nt4Table.getIntegerTopic("mosi").publish();

		questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(0);
		questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
		questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]
		{ 0.0f, 0.0f, 0.0f });
		questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]
		{ 0.0f, 0.0f, 0.0f, 0.0f });
		questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]
		{ 0.0f, 0.0f, 0.0f });
		questBattery = nt4Table.getDoubleTopic("batteryLevel").subscribe(0.0f);
	}

	@Override
	public void updateInputs(OculusIOInputs inputs)
	{
		inputs.connected = nt4Instance.isConnected();
		inputs.getOculusPosition = getOculusPosition();
		inputs.getOculusYaw = getOculusYaw();
		inputs.getOculusPose = getOculusPose();
	}

	// Get the yaw Euler angle of the headset
	public float getOculusYaw()
	{
		float[] eulerAngles = questEulerAngles.get();
		var ret = eulerAngles[1];
		ret %= 360;
		if (ret < 0)
		{
			ret += 360;
		}
		return ret;
	}

	public Translation2d getOculusPosition()
	{
		float[] oculusPosition = questPosition.get();
		return new Translation2d(oculusPosition[2], -oculusPosition[0]);
	}

	public Pose2d getOculusPose()
	{
		var oculousPositionCompensated = getOculusPosition().minus(new Translation2d(0, 0.1651)); // 6.5
		return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
	}
}
