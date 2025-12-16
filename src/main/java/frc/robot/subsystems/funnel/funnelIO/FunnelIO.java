package frc.robot.subsystems.funnel.funnelIO;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class FunnelIO extends IOBase{
    
    public final BooleanSupplier isBeamBreak = fields.addBoolean("BeamBreak", this::getBeamBreak);
    public final DoubleSupplier getMotorCurrent = fields.addDouble("MotorCurrent", this::getMotorCurrent);
    
    public FunnelIO(LogFieldsTable logFieldsTable){
        super(logFieldsTable);
    }

    //Output methods:
    protected abstract boolean getBeamBreak();
    protected abstract double getMotorCurrent();

    //Input methods:
    public abstract void setPrecentageSpeed(double speed);
}
