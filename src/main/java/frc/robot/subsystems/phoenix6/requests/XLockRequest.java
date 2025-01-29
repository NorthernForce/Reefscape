package frc.robot.subsystems.phoenix6.requests;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class XLockRequest implements SwerveRequest
{

	@Override
	public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply)
	{
		for (int i = 0; i < modulesToApply.length; i++)
		{
			modulesToApply[i].apply(new SwerveModule.ModuleRequest()
					.withState(new SwerveModuleState(0, parameters.moduleLocations[i].getAngle())));
		}
		return StatusCode.OK;
	}
}
