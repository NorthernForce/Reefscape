package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.leds.LEDS;

public class ledsCommands {
    public static Command getSetColour(LEDS leds, int r, int g, int b) {

        return Commands.runOnce(() -> leds.setLEDColour(r, g, b), leds);
    }
}
