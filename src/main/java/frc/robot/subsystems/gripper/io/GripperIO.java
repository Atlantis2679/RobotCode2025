package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoraIn = fields.addBoolean("isCoraIn", this::getIsCoralIn);

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Outputs:
    public abstract void setRightOuttakeMotorVoltage(double voltage);
    public abstract void setLeftOuttakeMotorVoltage(double voltage);
    public abstract void setBackMotorVoltage(double voltage);

    // Intputs:
    protected abstract boolean getIsCoralIn();
    protected abstract double getRightOuttakeMotorVoltage();
    protected abstract double getRightOuttakeMotorCurrent();
    protected abstract double getLeftOuttakeMotorVoltage();
    protected abstract double getLeftOuttakeMotorCurrent();
    protected abstract double getBackMotorVoltage();
    protected abstract double getBackMotorCurrent();

    protected abstract REVLibError getRightOuttakeMotorConfigError();
    protected abstract REVLibError getLeftOuttakeMotorConfigError();
    protected abstract REVLibError getBackMotorConfigError();

    protected abstract Faults getRightOuttakeMotorFaults();
    protected abstract Faults getLeftOuttakeMotorFaults();
    protected abstract Faults getBackMotorFaults();

    protected abstract Warnings getRightOuttakeMotorWarnings();
    protected abstract Warnings getLeftOuttakeMotorWarnings();
    protected abstract Warnings getBackMotorWarnings();
}
