package frc.robot.subsystems.pivot.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class PivotIO extends IOBase {
    public final DoubleSupplier motorCurrent = fields.addDouble("motor current", this::getMotorCurrent);
    public final DoubleSupplier angle = fields.addDouble("angle", this::getPivotAngleDegrees);
    public final BooleanSupplier isEncoderConnected = fields.addBoolean("isEncoderConnected", this::getIsEncoderConnected);

    public PivotIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    protected abstract double getMotorCurrent();

    protected abstract double getPivotAngleDegrees();
    
    protected abstract boolean getIsEncoderConnected();

    // Outputs:
    public abstract void setVoltage(double voltage);
}