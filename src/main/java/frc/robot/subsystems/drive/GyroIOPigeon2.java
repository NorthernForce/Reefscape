// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO
{
	// TODO: add yaw timestamps, pheonix stuff, etc
	private final Pigeon2 pigeon = new Pigeon2(20);
	private final StatusSignal<Angle> yaw = pigeon.getYaw();
	private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

	public GyroIOPigeon2(boolean phoenixDrive)
	{
		pigeon.getConfigurator().apply(new Pigeon2Configuration());
		pigeon.getConfigurator().setYaw(0.0);
		yaw.setUpdateFrequency(Module.ODOMETRY_FREQUENCY);
		yawVelocity.setUpdateFrequency(100.0);
		pigeon.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(GyroIOInputs inputs)
	{
		inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
		inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
		inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
	}
}