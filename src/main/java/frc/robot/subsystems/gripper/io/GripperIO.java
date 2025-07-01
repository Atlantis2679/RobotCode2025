package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoralIn = fields.addBoolean("isCoralIn", this::getIsCoralIn);
    public final DoubleSupplier rightOuttakeMotorCurrent = fields.addDouble("rightOuttakeMotorCurrect",
            this::getRightOuttakeMotorCurrent);
    public final DoubleSupplier leftOuttakeMotorCurrent = fields.addDouble("leftOuttakeMotorCurrent",
            this::getLeftOuttakeMotorCurrent);
    public final DoubleSupplier backMotorCurrent = fields.addDouble("backMotorCurrent", this::getBackMotorCurrent);

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    
    protected abstract boolean getIsCoralIn();
    
    protected abstract double getRightOuttakeMotorCurrent();
    
    protected abstract double getLeftOuttakeMotorCurrent();
    
    protected abstract double getBackMotorCurrent();

    // Outputs:

    public abstract void setBreakMotor(boolean isBreak);

    public abstract void setRightOuttakeMotorVoltage(double voltage);

    public abstract void setLeftOuttakeMotorVoltage(double voltage);

    public abstract void setBackMotorVoltage(double voltage);
}
