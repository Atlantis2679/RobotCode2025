package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LedsCommands {
    private final Leds leds;

    public LedsCommands(Leds leds) {
        this.leds = leds;
    }

    // public Command rainbow() {
    //     // return Commands.runOnce(leds.applyPatern(LEDPattern.rainbow(0, 0));, null)
    // }
}
