package frc.robot.subsystems.pivot.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class PivotIO extends IOBase {
    public final DoubleSupplier leftMotorCurrent = fields.addDouble("Left motor current", this::getLeftMotorCurrent);
    public final DoubleSupplier rightMotorCurrent = fields.addDouble("Right motor current", this::getRightMotorCurrent);
    public final DoubleSupplier leftMotorVoltage = fields.addDouble("Left motor voltage", this::getLeftMotorVoltage);
    public final DoubleSupplier rightMotorVoltage = fields.addDouble("Right motor voltage", this::getRightMotorVoltage);
    public final DoubleSupplier angle = fields.addDouble("Pivot angle", this::getPivotAngleDegrees);

    public PivotIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // outputs: 
    protected abstract double getLeftMotorCurrent();
    protected abstract double getRightMotorCurrent();
    protected abstract double getLeftMotorVoltage();
    protected abstract double getRightMotorVoltage();
    protected abstract double getPivotAngleDegrees();
    
    // inputs: 
    public abstract void setVoltage(double voltage);
}
