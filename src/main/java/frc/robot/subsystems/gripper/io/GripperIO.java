package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoraIn = fields.addBoolean("isCoraIn", this::getIsCoralIn);
    public final LongSupplier rightOuttakeMotorStatusValue = fields.addInteger(
        "rightOuttakeMotorStatusValue", this::getRightOuttakeMotorStatusValue);
    public final LongSupplier leftOuttakeMotorStatusValue = fields.addInteger(
    "leftOuttakeMotorStatusValue", this::getLeftOuttakeMotorStatusValue);
    public final DoubleSupplier leftMotorVoltage = fields.addDouble(
        "leftMotorVoltage", this::getLeftMotorVoltage);
    public final DoubleSupplier rightMotorVoltage = fields.addDouble(
        "rightMotorVoltage", this::getRightMotorVoltage);

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Intputs:
    protected abstract boolean getIsCoralIn();

    protected abstract int getRightOuttakeMotorStatusValue();
    protected abstract int getLeftOuttakeMotorStatusValue();
    protected abstract double getLeftMotorVoltage();
    protected abstract double getRightMotorVoltage();

    // Outputs:
    public abstract void setRightMotorVoltage(double voltage);
    public abstract void setLeftMotorVoltage(double voltage);
}
