package frc.robot.zippy.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.blenny.constants.BlennyTunerConstants;
import frc.robot.subsystems.leds.LedsIO;

public class ZippyConstants
{
    public static class DrivetrainConstants
    {
        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.7);
        public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(3.0);
        public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(0.7);
        public static final Angle[] SWERVE_MODULE_OFFSETS =
        { Rotations.of(Preferences.getDouble("kSwerveOffestFrontLeft", BlennyTunerConstants.FrontLeft.EncoderOffset)),
                Rotations.of(Preferences.getDouble("kSwerveOffestFrontRight",
                        BlennyTunerConstants.FrontRight.EncoderOffset)),
                Rotations.of(
                        Preferences.getDouble("kSwerveOffestBackLeft", BlennyTunerConstants.BackLeft.EncoderOffset)),
                Rotations.of(Preferences.getDouble("kSwerveOffestBackRight",
                        BlennyTunerConstants.BackRight.EncoderOffset)) };
    }

    public static class LedConstants
    {
        public static int CanID = 30;
        public static LedsIO.LedConstantsRecord ledInputs = new LedsIO.LedConstantsRecord(85, 0.5, 0.1, false, 0);
    }
}
