package frc.robot.subsystems.leds;

import java.time.Instant;
import java.time.temporal.TemporalField;
import java.util.Date;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase
{
    private int timeStamp = (int) Instant.now().getEpochSecond();
    private final LedsIO io;
    LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

    public LEDS(LedsIO io)
    {
        this.io = io;
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    public void setLEDColour(int r, int g, int b)
    {
        io.setColours(r, g, b);
    }

    public void switchLEDs(boolean switched)
    {
        io.setOn(switched);
    }

    public void setLEDBrightness(double brightness)
    {
        io.setBrightness(brightness);
    }

    public void setSpecificLEDs(int startIdx, int endIdx, int r, int g, int b)
    {
        io.setSpecificLEDs(startIdx, endIdx, r, g, b);
    }

    public void rainbowAnimation()
    {
        io.rainbowAnimation();
    }

    public void twinkleAnimation(int r, int g, int b)
    {
        io.twinkleAnimation(r, g, b);
    }

    public void colourFlow(int r, int g, int b, boolean direction, int offSet)
    {
        io.colourFlow(r, g, b, direction, offSet);
    }

    public void strobeAnimation(int r, int g, int b)
    {
        io.strobeAnimation(r, g, b);
    }

    public void incrementAnimation()
    {
        io.incrementAnimation();
    }

    public void clearAnimationBuffer()
    {
        io.clearAnimationBuffer();
    }

    public void compassEffect(Angle degrees)
    {
        io.compassEffect(degrees);
    }

    public void lightList(int[] leds, int r, int g, int b)
    {
        io.lightList(leds, r, g, b);
    }

    public Command getSetColour(int r, int g, int b)
    {
        run(() -> clearAnimationBuffer());
        SmartDashboard.putNumber("Second", (Instant.now().getEpochSecond()));
        SmartDashboard.putBoolean("B button", !SmartDashboard.getBoolean("B button", false));
        return run(() -> setLEDColour(r, g, b));
    }

    public Command getRainbowAnimation()
    {
        return run(() -> rainbowAnimation());
    }

    public Command getIncrementAnimation()
    {
        return run(() -> incrementAnimation());
    }
}
