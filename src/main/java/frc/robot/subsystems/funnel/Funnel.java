package frc.robot.subsystems.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.Robot;
import frc.robot.subsystems.funnel.io.FunnelIO;
import frc.robot.subsystems.funnel.io.FunnelIOSim;
import frc.robot.subsystems.funnel.io.FunnelIOSparksMax;

import static frc.robot.subsystems.funnel.FunnelConstants.*;

public class Funnel extends SubsystemBase {
    private final FunnelIO io;
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final Debouncer isCoralInDebouncer = new Debouncer(DEBOUNCER_SECONDS, DebounceType.kBoth);

    private final Alert isCoralInAlert = new Alert("Funnel", "Coral Not Inside", AlertType.kWarning);

    public Funnel() {
        io = Robot.isReal() ? new FunnelIOSparksMax(this.fieldsTable) : new FunnelIOSim(this.fieldsTable);
    }

    @Override
    public void periodic() {
        isCoralInAlert.set(io.isCoralIn.getAsBoolean());
    }

    public void setMotorVoltage(double voltageDemand) {
        io.setVoltage(MathUtil.clamp(voltageDemand, -MOTOR_MAX_VOLTAGE, MOTOR_MAX_VOLTAGE));
    }

    public void setMotorPercentageSpeed(double percentageSpeed)  {
        io.setPercentageSpeed(MathUtil.clamp(percentageSpeed, -1, 1));
    }

    public boolean getIsCoralIn() {
        return isCoralInDebouncer.calculate(io.isCoralIn.getAsBoolean());
    }

    public void stop() {
        io.setPercentageSpeed(0);
    }
}
