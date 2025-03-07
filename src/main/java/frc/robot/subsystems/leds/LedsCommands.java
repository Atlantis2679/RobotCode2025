package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.subsystems.leds.LedsConstants.*;

import java.util.function.BooleanSupplier;

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

    public Command staticColorWhenTrue(BooleanSupplier condition, Color color) {
        return Commands.waitUntil(condition)
                .andThen(staticColor(Color.kGreen))
                .until(() -> !condition.getAsBoolean()).repeatedly();
    }

    public Command rainbow() {
        return leds.startEnd(() -> {
            leds.applyPatern(LEDPattern.rainbow(RAINBOW_SATURATION, RAINBOW_VALUE)
                    .scrollAtRelativeSpeed(Percent.per(Second).of(50))
                    .atBrightness(Percent.of(60)));
        }, leds::clear);
    }

    public Command bebeGradient() {
        return leds.startEnd(() -> {
            leds.applyPatern(LEDPattern
                    .gradient(GradientType.kContinuous,
                            new Color("#091a79"),
                            new Color("#00bebe"),
                            new Color("#00c21e"))
                    .scrollAtRelativeSpeed(Percent.per(Second).of(50))
                    .atBrightness(Percent.of(60)));
        }, leds::clear);
    }
}
