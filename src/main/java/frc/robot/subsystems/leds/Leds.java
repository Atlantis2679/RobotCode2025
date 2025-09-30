package frc.robot.subsystems.leds;

import static frc.robot.RobotMap.LEDS_ID;
import static frc.robot.subsystems.leds.LedsConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Leds extends SubsystemBase {
    AddressableLED leds = new AddressableLED(LEDS_ID);

    AddressableLEDBuffer ledsBuffer = new AddressableLEDBuffer(LEDS_LENGHT);

    LEDPattern lastAppliedPattern = LEDPattern.solid(Color.kBlack);

    LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    public Leds() {
        leds.setLength(LEDS_LENGHT);
        leds.start();
    }

    @Override
    public void periodic() {
        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        lastAppliedPattern.applyTo(ledsBuffer);
        leds.setData(ledsBuffer);
    }

    public void applyPattern(LEDPattern ledsPattern) {
        lastAppliedPattern = ledsPattern;
    }

    public void applyColor(Color color) {
        applyPattern(LEDPattern.solid(color));
    }

    public void clear() {
        applyColor(Color.kBlack);
    }
}
