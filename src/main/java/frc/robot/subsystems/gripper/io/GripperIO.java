package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoralIn = fields.addBoolean("isCoralIn", this::getIsCoralIn);
    public final DoubleSupplier rightMotorCurrent = fields.addDouble("rightMotorCurrect",
            this::getRightMotorCurrent);
    public final DoubleSupplier leftMotorCurrent = fields.addDouble("leftMotorCurrent",
            this::getLeftMotorCurrent);
    public final DoubleSupplier backMotorCurrent = fields.addDouble("backMotorCurrent", this::getBackMotorCurrent);

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    
    protected abstract boolean getIsCoralIn();
    
    protected abstract double getRightMotorCurrent();
    
    protected abstract double getLeftMotorCurrent();
    
    protected abstract double getBackMotorCurrent();

    // Outputs:

    public abstract void setBreakMotor(boolean isBreak);

    public abstract void setRightMotorVoltage(double voltage);

    public abstract void setLeftMotorVoltage(double voltage);

    public abstract void setBackMotorVoltage(double voltage);
}
