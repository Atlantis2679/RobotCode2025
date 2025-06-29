package frc.robot.subsystems.swerve.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import atlantis2679.lib.logfields.LogFieldsTable;
import atlantis2679.lib.logfields.IOBase;

public abstract class GyroIO extends IOBase {
    public final DoubleSupplier yawDegreesCW = fields.addDouble("yawDegreesCW", this::getYawDegreesCW);
    public final BooleanSupplier isConnected = fields.addBoolean("isConnected", this::getIsConnected);
    
    public GyroIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputes

    protected abstract double getYawDegreesCW();

    protected abstract boolean getIsConnected();
}
