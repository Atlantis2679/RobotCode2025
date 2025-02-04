package frc.robot.subsystems.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

    private double lastDemandVoltage = 0;
    private boolean lastDebouncerValue = false;

    public Funnel() {
        io = Robot.isReal() ? new FunnelIOSparksMax(this.fieldsTable) : new FunnelIOSim(this.fieldsTable);
    }

    @Override
    public void periodic() {
        fieldsTable.recordOutput("right motor real voltage", io.rightVoltage.getAsDouble());
        fieldsTable.recordOutput("left motor real voltage", io.leftVoltage.getAsDouble());
        fieldsTable.recordOutput("motors voltage demand", lastDemandVoltage);
        fieldsTable.recordOutput("last debauncer value", getIsCoralIn());
    }

    public void setMotorVoltage(double voltageDemand) {
        lastDemandVoltage = voltageDemand;
        io.setVoltage(MathUtil.clamp(voltageDemand, -MOTORS_MAX_VOLTAGE, MOTORS_MAX_VOLTAGE));
    }

    public void setMotorPercentageSpeed(double percentageSpeed)  {
        lastDemandVoltage = percentageSpeed * MOTORS_MAX_VOLTAGE;
        io.setPercentageSpeed(MathUtil.clamp(percentageSpeed, -1, 1));
    }

    public boolean getIsCoralIn() {
        lastDebouncerValue = isCoralInDebouncer.calculate(io.isCoralIn.getAsBoolean());
        return lastDebouncerValue;
    }

    public void stop() {
        io.setPercentageSpeed(0);
    }
}
