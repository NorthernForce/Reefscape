package frc.robot.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DrivePositionMeters", drivePositionMeters);
    table.put("DriveVelocityMetersPerSecond", driveVelocityMetersPerSecond);
    table.put("DriveAppliedVolts", driveAppliedVolts);
    table.put("DriveCurrentAmps", driveCurrentAmps);
    table.put("TurnPosition", turnPosition);
    table.put("TurnVelocityRotationsPerSecond", turnVelocityRotationsPerSecond);
    table.put("TurnAppliedVolts", turnAppliedVolts);
    table.put("TurnCurrentAmps", turnCurrentAmps);
    table.put("OdometryTimestamps", odometryTimestamps);
    table.put("OdometryDrivePositionsMeters", odometryDrivePositionsMeters);
    table.put("OdometryTurnPositions", odometryTurnPositions);
    table.put("DriveTemperature", driveTemperature);
    table.put("TurnTemperature", turnTemperature);
  }

  @Override
  public void fromLog(LogTable table) {
    drivePositionMeters = table.get("DrivePositionMeters", drivePositionMeters);
    driveVelocityMetersPerSecond = table.get("DriveVelocityMetersPerSecond", driveVelocityMetersPerSecond);
    driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
    driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);
    turnPosition = table.get("TurnPosition", turnPosition);
    turnVelocityRotationsPerSecond = table.get("TurnVelocityRotationsPerSecond", turnVelocityRotationsPerSecond);
    turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
    turnCurrentAmps = table.get("TurnCurrentAmps", turnCurrentAmps);
    odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
    odometryDrivePositionsMeters = table.get("OdometryDrivePositionsMeters", odometryDrivePositionsMeters);
    odometryTurnPositions = table.get("OdometryTurnPositions", odometryTurnPositions);
    driveTemperature = table.get("DriveTemperature", driveTemperature);
    turnTemperature = table.get("TurnTemperature", turnTemperature);
  }

  public ModuleIOInputsAutoLogged clone() {
    ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
    copy.drivePositionMeters = this.drivePositionMeters;
    copy.driveVelocityMetersPerSecond = this.driveVelocityMetersPerSecond;
    copy.driveAppliedVolts = this.driveAppliedVolts;
    copy.driveCurrentAmps = this.driveCurrentAmps.clone();
    copy.turnPosition = this.turnPosition;
    copy.turnVelocityRotationsPerSecond = this.turnVelocityRotationsPerSecond;
    copy.turnAppliedVolts = this.turnAppliedVolts;
    copy.turnCurrentAmps = this.turnCurrentAmps.clone();
    copy.odometryTimestamps = this.odometryTimestamps.clone();
    copy.odometryDrivePositionsMeters = this.odometryDrivePositionsMeters.clone();
    copy.odometryTurnPositions = this.odometryTurnPositions.clone();
    copy.driveTemperature = this.driveTemperature;
    copy.turnTemperature = this.turnTemperature;
    return copy;
  }
}
