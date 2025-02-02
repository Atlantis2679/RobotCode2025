package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoraIn = fields.addBoolean("isCoraIn", this::getIsCoralIn);
    public final LongSupplier rightOuttakeMotorStatusValue = fields.addInteger(
        "rightOuttakeMotorStatusValue", this::getRightOuttakeMotorStatusValue);
    public final LongSupplier leftOuttakeMotorStatusValue = fields.addInteger(
    "leftOuttakeMotorStatusValue", this::getLeftOuttakeMotorStatusValue);
    public final LongSupplier backMotorStatusValue = fields.addInteger(
        "backMotorStatusValue", this::getBackMotorStatusValue);
    

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Intputs:
    protected abstract boolean getIsCoralIn();

    protected abstract int getRightOuttakeMotorStatusValue();
    protected abstract int getLeftOuttakeMotorStatusValue();
    protected abstract int getBackMotorStatusValue();

    // Outputs:
    public abstract void setRightOuttakeMotorVoltage(double voltage);
    public abstract void setLeftOuttakeMotorVoltage(double voltage);
    public abstract void setBackMotorVoltage(double voltage);

}
