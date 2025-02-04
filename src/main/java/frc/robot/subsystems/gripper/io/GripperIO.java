package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoraIn = fields.addBoolean("isCoraIn", this::getIsCoralIn);
    public final DoubleSupplier leftOuttakeMotorVoltage = fields.addDouble(
        "leftMotorVoltage", this::getLeftMotorVoltage);
    public final DoubleSupplier rightOuttakeMotorVoltage = fields.addDouble(
        "rightMotorVoltage", this::getRightMotorVoltage);

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Intputs:
    protected abstract boolean getIsCoralIn();
    protected abstract double getLeftMotorVoltage();
    protected abstract double getRightMotorVoltage();

    // Outputs:
    public abstract void setRightMotorVoltage(double voltage);
    public abstract void setLeftMotorVoltage(double voltage);

}
