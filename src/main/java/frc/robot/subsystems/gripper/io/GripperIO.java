package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoraIn = fields.addBoolean("isCoraIn", this::getIsCoralIn);
    public final DoubleSupplier leftOutTakeMotorSpeedRPM = fields.addDouble("leftOutTakeMotorSpeedRPM", this::getLeftOutTakeMotorSpeedRPM);
    public final DoubleSupplier rightOutTakeMotorSpeedRPM = fields.addDouble("rightOutTakeMotorSpeedRPM", this::getRightOutTakeMotorSpeedRPM);
    public final DoubleSupplier intakeOutTakeMotorSpeedRPM = fields.addDouble("intakeOutTakeMotorSpeedRPM", this::getIntakeOutTakeMotorSpeedRPM);

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // intputs:
    protected abstract boolean getIsCoralIn();
    protected abstract double getRightOutTakeMotorSpeedRPM();
    protected abstract double getLeftOutTakeMotorSpeedRPM();
    protected abstract double getIntakeOutTakeMotorSpeedRPM();

    // outputs:
    public abstract void setVoltageRightOutTake(double voltage);
    public abstract void setPrecentageSpeedRightOutTake(double precentageSpeed);
    public abstract void setVoltageLeftOutTake(double voltage);
    public abstract void setPrecentageSpeedLeftOutTake(double precentageSpeed);
    public abstract void setVoltageIntakeOutTake(double voltage);
    public abstract void setPrecentageSpeedIntakeOutTake(double precentageSpeed);

}
