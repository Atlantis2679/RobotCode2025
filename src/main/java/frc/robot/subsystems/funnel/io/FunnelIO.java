package frc.robot.subsystems.funnel.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class FunnelIO extends IOBase {
    public final BooleanSupplier isCoralIn = fields.addBoolean("isCoralIn", this::getIsCoralIn);
    public final DoubleSupplier motorVoltage = fields.addDouble("motorVoltage", this::getVoltage);
    public final DoubleSupplier motorCurrent = fields.addDouble("motorCurrent", this::getCurrent);

    public FunnelIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    public abstract void setVoltage(double voltageDemand);
    public abstract void setPercentageSpeed(double percentageSpeed);

    // Outputs:
    protected abstract boolean getIsCoralIn();
    protected abstract double getVoltage();
    protected abstract double getCurrent();

}
