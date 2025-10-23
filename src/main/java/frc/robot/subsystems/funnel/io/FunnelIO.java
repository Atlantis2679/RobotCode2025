package frc.robot.subsystems.funnel.io;

import java.util.function.BooleanSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class FunnelIO extends IOBase {
    public BooleanSupplier beamBreak = fields.addBoolean("beamBreak", this::getBeamBreak);

    public FunnelIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }
    
    protected abstract boolean getBeamBreak();
    public abstract void setVoltage(double voltage);
}
