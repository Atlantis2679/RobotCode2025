package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    public static Leds[] LED_STRIPS = new Leds[0];
    public static final AddressableLEDBuffer ledBuffer = LedsConstants.LED_BUFFER;
    private static final AddressableLED led = LedsConstants.LED;
    private final int numberOfLeds;
    private final int ledsOffsetIndex;
    private boolean areLEDsOnForBlinking = false;
    private double lastBlinkTime = 0;

    public Leds(int numberOfLeds, int ledsOffsetIndex) {
        this.numberOfLeds = numberOfLeds;
        this.ledsOffsetIndex = ledsOffsetIndex;

        addLEDStripToLEDStripsArrayArray(this);
    }

    public int getNumberOfLeds(){
        return numberOfLeds;
    }
        public static void setDefaultCommandForAllLEDS(Command command) {
        for (Leds ledStrip : LED_STRIPS)
            ledStrip.setDefaultCommand(command);
    }

    void clearLedColors() {
        staticColor(Color.kBlack);
    }
    void staticColor(Color color) {
        for (int index = 0; index < numberOfLeds-1; index++) {
            LedsConstants.LED_BUFFER.setLED(index + ledsOffsetIndex, color);
        }
        led.setData(LedsConstants.LED_BUFFER);
    }
    void blink(Color color, double blinkingIntervalSeconds) {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastBlinkTime > blinkingIntervalSeconds) {
            lastBlinkTime = currentTime;
            areLEDsOnForBlinking = !areLEDsOnForBlinking;
        }
        if (areLEDsOnForBlinking)
            staticColor(color);
        else
            clearLedColors();
    }
    public boolean ledsTimeOut(){
        Commands.waitSeconds(5);
        return true;
    }
    private void addLEDStripToLEDStripsArrayArray(Leds ledStrip) {
        final Leds[] newLEDStrips = new Leds[LED_STRIPS.length + 1];
        System.arraycopy(LED_STRIPS, 0, newLEDStrips, 0, LED_STRIPS.length);
        newLEDStrips[LED_STRIPS.length] = ledStrip;
        LED_STRIPS = newLEDStrips;
    }
}