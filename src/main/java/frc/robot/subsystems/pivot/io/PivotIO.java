package frc.robot.subsystems.pivot.io;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class PivotIO extends IOBase {
    public final DoubleSupplier motorCurrent = fields.addDouble("motor current", this::getMotorCurrent);
    public final DoubleSupplier motorVoltage = fields.addDouble("motor voltage", this::getMotorVoltage);
    public final DoubleSupplier angle = fields.addDouble("Pivot angle", this::getPivotAngleDegrees);
    public final Supplier<REVLibError> motorConfigError = fields.addREVLibError(
        "motorConfigError", this::getMotorConfigError);
    public final Supplier<Faults> motorFaults = fields.addSparkFaults("motorFaults", this::getMotorFaults);
    public final Supplier<Warnings> motorWarnings = fields.addSparkWarnings("motorWarnings", this::getMotorWarnings);

    public PivotIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Outputs: 
    protected abstract double getMotorCurrent();
    protected abstract double getMotorVoltage();
    protected abstract double getPivotAngleDegrees();

    protected abstract REVLibError getMotorConfigError();

    protected abstract Faults getMotorFaults();

    protected abstract Warnings getMotorWarnings();
    
    // Inputs: 
    public abstract void setVoltage(double voltage);
}