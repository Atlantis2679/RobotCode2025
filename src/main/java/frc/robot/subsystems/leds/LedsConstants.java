package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotMap;

public class LedsConstants {
    public final static int TOTAL_LEDS_NUMBER = 74;

    static final AddressableLEDBuffer LED_BUFFER = new AddressableLEDBuffer(TOTAL_LEDS_NUMBER);
    static final AddressableLED LED = new AddressableLED(RobotMap.LEDS_ID);
    
    public static final Leds
            RIGHT_CLIMBER_LEDS = new Leds(TOTAL_LEDS_NUMBER, 0);
            // LEFT_CLIMBER_LEDS = new Leds(LEDS_LEFT_LENGTH, LEDS_RIGHT_LENGTH - 1);
    
    static {
        LED.setLength(TOTAL_LEDS_NUMBER);
        LED.setData(LED_BUFFER);
        LED.start();
    }
}