package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;

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
}
