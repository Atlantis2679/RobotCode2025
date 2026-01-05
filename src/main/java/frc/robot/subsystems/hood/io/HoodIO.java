
package frc.robot.subsystems.hood.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class HoodIO extends IOBase {
    public final DoubleSupplier motorCurrent = fields.addDouble("motor current", this::getMotorCurrent);
    public final DoubleSupplier angle = fields.addDouble("angle", this::getHoodAngleDegrees);
    public final BooleanSupplier isEncoderConnected = fields.addBoolean("isEncoderConnected", this::getIsEncoderConnected);

    public HoodIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    protected abstract double getMotorCurrent();

    protected abstract double getHoodAngleDegrees();
    
    protected abstract boolean getIsEncoderConnected();

    // Outputs:
    public abstract void setVoltage(double voltage);
}