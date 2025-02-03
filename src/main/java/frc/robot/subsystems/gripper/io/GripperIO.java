package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoraIn = fields.addBoolean("isCoraIn", this::getIsCoralIn);
    public final DoubleSupplier leftOuttakeMotorVoltage = fields.addDouble(
        "leftOuttakeMotorVoltage", this::getBackMotorVoltage);
    public final DoubleSupplier backMotorVoltage = fields.addDouble(
        "backMotorVoltage", this::getLeftOuttakeMotorVoltage);    
    public final DoubleSupplier rightOuttakeMotorVoltage = fields.addDouble(
        "rightOuttakeMotorVoltage", this::getRightOuttakeMotorVoltage);

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Intputs:
    protected abstract boolean getIsCoralIn();
    protected abstract double getBackMotorVoltage();
    protected abstract double getLeftOuttakeMotorVoltage();
    protected abstract double getRightOuttakeMotorVoltage();

    // Outputs:
    public abstract void setRightMotorVoltage(double voltage);
    public abstract void setLeftMotorVoltage(double voltage);

}
