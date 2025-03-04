package frc.robot.subsystems.leds;

import static frc.robot.RobotMap.LEDS_ID;
import static frc.robot.subsystems.leds.LedsConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    AddressableLED leds = new AddressableLED(LEDS_ID);

    AddressableLEDBuffer ledsBuffer = new AddressableLEDBuffer(LEDS_LENGHT);

    LEDPattern lastAppliedPatern = LEDPattern.solid(Color.kBlack);

    public Leds() {
        leds.setLength(LEDS_LENGHT);
        leds.start();
    }

    @Override
    public void periodic() {
        lastAppliedPatern.applyTo(ledsBuffer);
        leds.setData(ledsBuffer);
    }

    public void applyPatern(LEDPattern ledsPattern) {
        lastAppliedPatern = ledsPattern;
    }

    public void applyColor(Color color) {
        applyPatern(LEDPattern.solid(color));
    }

    public void clear() {
        applyColor(Color.kBlack);
    }
}
