package frc.robot.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Dashboard {
  private static NetworkTable table;

  public void DashboardNetworkTable() {
    table = NetworkTableInstance.getDefault().getTable("Dashboard");
    periodicUpdate();
  }

  public void putString(String key, String value) {
    table.getEntry(key).setString(value);
  }

  public void periodicUpdate() {
    while (true) {
      table.getEntry("heartbeat").setString("alive");
      try {
        Thread.sleep(200); // delay for 1 second
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }
  }
}
