package frc.robot.subsystems.funnel.io;

import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class FunnelIO extends IOBase {
    public final BooleanSupplier isCoralIn = fields.addBoolean("isCoralIn", this::getIsCoralIn);
    public final LongSupplier motorStatusValue = fields.addInteger("motorStatusValue", this::getMotorStatusValue);
    public final DoubleSupplier leftVoltage = fields.addDouble("leftVoltage", this::getLeftVoltage);
    public final DoubleSupplier rightVoltage = fields.addDouble("rightVoltage", this::getRightVoltage);

    public FunnelIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    public abstract void setVoltage(double voltageDemand);
    public abstract void setPercentageSpeed(double percentageSpeed);

    // Outputs:
    protected abstract boolean getIsCoralIn();
    protected abstract int getMotorStatusValue();
    protected abstract double getLeftVoltage();
    protected abstract double getRightVoltage();
}
