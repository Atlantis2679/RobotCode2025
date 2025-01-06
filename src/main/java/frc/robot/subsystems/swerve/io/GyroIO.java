package frc.robot.subsystems.swerve.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.logfields.IOBase;

public abstract class GyroIO extends IOBase{
    public final DoubleSupplier yawDegreesCW = fields.addDouble("yawDegreesCW", this::getYawDegreesCW);
    public final BooleanSupplier isConnected = fields.addBoolean("isConnected", this::getIsConnected);
    public final BooleanSupplier isMoving = fields.addBoolean("isMoving", this::getIsMoving);
    
    public GyroIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputes

    protected abstract double getYawDegreesCW();

    protected abstract boolean getIsConnected();

    protected abstract boolean getIsMoving();
}
