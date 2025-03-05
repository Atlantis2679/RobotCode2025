package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.subsystems.leds.LedsConstants.*;

public class LedsCommands {
    private final Leds leds;

    public LedsCommands(Leds leds) {
        this.leds = leds;
    }

    public Command blink(Color color, double seconds) {
        return leds.runOnce(() -> leds.applyColor(color))
                .andThen(Commands.waitSeconds(seconds))
                .finallyDo(leds::clear);
    }

    public Command staticColor(Color color) {
        return leds.startEnd(() -> {
            leds.applyColor(color);
        }, leds::clear);
    }

    public Command rainbow(double seconds) {
        return leds.runOnce(() -> leds.applyPatern(LEDPattern.rainbow(RAINBOW_SATURATION, RAINBOW_VALUE)))
                .andThen(Commands.waitSeconds(seconds))
                .finallyDo(leds::clear);
    }
}
