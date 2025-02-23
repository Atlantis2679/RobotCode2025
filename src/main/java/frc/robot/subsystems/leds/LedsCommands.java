package frc.robot.subsystems.leds;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LedsCommands {

    public static Command colorForSeconds(Color color, double seconds, Leds... ledStrips) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> runForLEDs(ledStrip -> ledStrip.staticColor(color), ledStrips)),
            new WaitCommand(seconds),
            new InstantCommand(() -> {
                runForLEDs(ledStrip -> ledStrip.clearLedColors(), ledStrips);
            })
        );
    }
    public static Command getStaticColorCommand(Color color, Leds... ledStrips) {
        return new StartEndCommand(
                () -> runForLEDs((ledStrip -> ledStrip.staticColor(color)), ledStrips),
                () -> {
                },
                ledStrips
        ).ignoringDisable(true);
    }
    public static Command clearLeds(Leds...ledStrips){
        return new InstantCommand(() -> runForLEDs((ledStrip -> ledStrip.clearLedColors()), ledStrips));
    }
    public static Command getBlinkingCommand(Color color, double blinkingIntervalSeconds, Leds... ledStrips) {
        return new RunCommand(
                () -> runForLEDs((ledStrip -> ledStrip.blink(color, blinkingIntervalSeconds)), ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static void runForLEDs(Consumer<Leds> action, Leds... ledStrips) {
        for (Leds ledStrip : ledStrips)
            action.accept(ledStrip);
    }

}