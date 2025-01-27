package frc.robot.subsystems.pivot.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class PivotIO extends IOBase {
    public final DoubleSupplier speed = fields.addDouble("Pivot voltage", this::getSpeed);
    public final DoubleSupplier angle = fields.addDouble("Pivot speed", this::getPivotAngleDegrees);

    public PivotIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputs
    protected abstract double getSpeed();
    protected abstract double getPivotAngleDegrees();
    // outputs
    public abstract void setSpeed(double speed);
}
