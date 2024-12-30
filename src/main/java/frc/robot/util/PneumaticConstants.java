package frc.robot.util;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public record PneumaticConstants(PneumaticsModuleType type, double minPressure, double maxPressure) {

}
