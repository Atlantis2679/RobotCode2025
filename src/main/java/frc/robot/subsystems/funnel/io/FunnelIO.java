package frc.robot.subsystems.funnel.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class FunnelIO extends IOBase {
    public final BooleanSupplier isCoralIn = fields.addBoolean("isCoralIn", this::getIsCoralIn);
    public final DoubleSupplier motorVoltage = fields.addDouble("motorVoltage", this::getVoltage);
    public final DoubleSupplier motorCurrent = fields.addDouble("motorCurrent", this::getCurrent);
    public final Supplier<REVLibError> motorConfigError = fields.addREVLibError("motorConfigError", this::getConfigError);
    public final Supplier<Faults> motorFaults = fields.addSparkFaults("motorFaults", this::getSparkFaults);
    public final Supplier<Warnings> motorWarnings = fields.addSparkWarnings("motorFaults", this::getSparkWarnings);

    public FunnelIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    public abstract void setVoltage(double voltageDemand);
    public abstract void setPercentageSpeed(double percentageSpeed);

    // Outputs:
    protected abstract boolean getIsCoralIn();
    protected abstract double getVoltage();
    protected abstract double getCurrent();

    protected abstract REVLibError getConfigError();
    protected abstract Faults getSparkFaults();
    protected abstract Warnings getSparkWarnings();
}
