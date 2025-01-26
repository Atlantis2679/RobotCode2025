package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoraIn = fields.addBoolean("isCoraIn", this::getIsCoralIn);

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Intputs:
    protected abstract boolean getIsCoralIn();

    // Outputs:
    public abstract void setRightOutTakeMotorVoltage(double voltage);
    public abstract void setRightOutTakeMotorPrecentageSpeed(double precentageSpeed);
    public abstract void setLeftOutTakeMotorVoltage(double voltage);
    public abstract void setLeftOutTakeMotorPrecentageSpeed(double precentageSpeed);
    public abstract void setBackMotorVoltage(double voltage);
    public abstract void setBackMotorPrecentageSpeed(double precentageSpeed);

}
