package frc.robot.subsystems.pivot.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class PivotIO extends IOBase {
    public final DoubleSupplier motorCurrent = fields.addDouble("motor current", this::getMotorCurrent);
    public final DoubleSupplier motorVoltage = fields.addDouble("motor voltage", this::getMotorVoltage);
    public final DoubleSupplier angle = fields.addDouble("Pivot angle", this::getPivotAngleDegrees);

    public PivotIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Outputs:
    protected abstract double getMotorCurrent();

    protected abstract double getMotorVoltage();

    protected abstract double getPivotAngleDegrees();

    // Inputs:
    public abstract void setVoltage(double voltage);
}