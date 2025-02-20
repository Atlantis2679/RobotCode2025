package frc.robot.subsystems.pivot.io;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class PivotIO extends IOBase {
    public final DoubleSupplier leftMotorCurrent = fields.addDouble("Left motor current", this::getLeftMotorCurrent);
    public final DoubleSupplier rightMotorCurrent = fields.addDouble("Right motor current", this::getRightMotorCurrent);
    public final DoubleSupplier leftMotorVoltage = fields.addDouble("Left motor voltage", this::getLeftMotorVoltage);
    public final DoubleSupplier rightMotorVoltage = fields.addDouble("Right motor voltage", this::getRightMotorVoltage);
    public final DoubleSupplier angle = fields.addDouble("Pivot angle", this::getPivotAngleDegrees);
    public final Supplier<REVLibError> leftMotorConfigError = fields.addREVLibError(
        "leftMotorConfigError", this::getLeftMotorConfigError);
    public final Supplier<REVLibError> rightMotorConfigError = fields.addREVLibError(
        "rightMotorConfigError", this::getRightMotorConfigError);
    public final Supplier<Faults> leftMotorFaults = fields.addSparkFaults("leftMotorFaults", this::getLeftMotorFaults);
    public final Supplier<Faults> rightMotorFaults = fields.addSparkFaults("rightMotorFaults", this::getRightMotorFaults);
    public final Supplier<Warnings> leftMotorWarnings = fields.addSparkWarnings("leftMotorWarnings", this::getLeftMotorWarnings);
    public final Supplier<Warnings> rightMotorWarnings = fields.addSparkWarnings("rightMotorWarnings", this::getRightMotorWarnings);

    public PivotIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Outputs: 
    protected abstract double getLeftMotorCurrent();
    protected abstract double getRightMotorCurrent();
    protected abstract double getLeftMotorVoltage();
    protected abstract double getRightMotorVoltage();
    protected abstract double getPivotAngleDegrees();

    protected abstract REVLibError getLeftMotorConfigError();
    protected abstract REVLibError getRightMotorConfigError();

    protected abstract Faults getLeftMotorFaults();
    protected abstract Faults getRightMotorFaults();

    protected abstract Warnings getLeftMotorWarnings();
    protected abstract Warnings getRightMotorWarnings();
    
    // Inputs: 
    public abstract void setVoltage(double voltage);
}