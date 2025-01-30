package frc.robot.subsystems.pivot.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class PivotIO extends IOBase {
    public final DoubleSupplier current = fields.addDouble("Pivot current", this::getCurrent);
    public final DoubleSupplier angle = fields.addDouble("Pivot angle", this::getPivotAngleDegrees);

    public PivotIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // outputs
    protected abstract double getCurrent();
    protected abstract double getPivotAngleDegrees();
    // inputs
    public abstract void setVoltage(double voltage);
}
