package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoraIn = fields.addBoolean("isCoraIn", this::getIsCoralIn);
    public final DoubleSupplier leftOutTakeMotorSpeedRPM = fields.addDouble("leftOutTakeMotorSpeedRPM", this::getLeftOutTakeMotorSpeedRPM);
    public final DoubleSupplier rightOutTakeMotorSpeedRPM = fields.addDouble("rightOutTakeMotorSpeedRPM", this::getRightOutTakeMotorSpeedRPM);
    public final DoubleSupplier backMotorSpeedRPM = fields.addDouble("backMotorSpeedRPM", this::getBackMotorSpeedRPM);

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Intputs:
    protected abstract boolean getIsCoralIn();
    protected abstract double getRightOutTakeMotorSpeedRPM();
    protected abstract double getLeftOutTakeMotorSpeedRPM();
    protected abstract double getBackMotorSpeedRPM();

    // Outputs:
    public abstract void setRightOutTakeMotorVoltage(double voltage);
    public abstract void setRightOutTakeMotorPrecentageSpeed(double precentageSpeed);
    public abstract void setLeftOutTakeMotorVoltage(double voltage);
    public abstract void setLeftOutTakeMotorPrecentageSpeed(double precentageSpeed);
    public abstract void setBackMotorVoltage(double voltage);
    public abstract void setBackMotorPrecentageSpeed(double precentageSpeed);

}
