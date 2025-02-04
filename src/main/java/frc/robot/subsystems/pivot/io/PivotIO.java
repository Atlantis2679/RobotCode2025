package frc.robot.subsystems.pivot.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class PivotIO extends IOBase {
    public final DoubleSupplier leftMotorcurrent = fields.addDouble("Left motor current", this::getLeftMotorCurrent);
    public final DoubleSupplier rightMotorcurrent = fields.addDouble("Right motor current", this::getRightMotorCurrent);
    public final DoubleSupplier angle = fields.addDouble("Pivot angle", this::getPivotAngleDegrees);

    public PivotIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // outputs: 
    protected abstract double getLeftMotorCurrent();
    protected abstract double getRightMotorCurrent();
    protected abstract double getPivotAngleDegrees();
    
    // inputs: 
    public abstract void setVoltage(double voltage);
}
