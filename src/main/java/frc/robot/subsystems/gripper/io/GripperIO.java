package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.GenericError;
import frc.lib.networkalerts.NetworkAlertsManager;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoralIn = fields.addBoolean("isCoralIn", this::getIsCoralIn);
    public final DoubleSupplier rightOuttakeMotorCurrent = fields.addDouble("rightOuttakeMotorCurrect",
            this::getRightOuttakeMotorCurrent);
    public final DoubleSupplier leftOuttakeMotorCurrent = fields.addDouble("leftOuttakeMotorCurrent",
            this::getLeftOuttakeMotorCurrent);
    public final DoubleSupplier backMotorCurrent = fields.addDouble("backMotorCurrent", this::getBackMotorCurrent);

    public final Supplier<GenericError> rightOuttakeMotorError = NetworkAlertsManager.addGenericError(
        fields.addGenericError("rightOuttakeMotorError", this::getRightOuttakeMotorError));

    public final Supplier<GenericError> leftOuttakeMotorError = NetworkAlertsManager.addGenericError(
        fields.addGenericError("leftOuttakeMotorError", this::getLeftOuttakeMotorError));

    public final Supplier<GenericError> backMotorError = NetworkAlertsManager.addGenericError(
        fields.addGenericError("backMotorError", this::getBackMotorError));

    public final Supplier<GenericError> rightOuttakeMotorWarning = NetworkAlertsManager.addGenericError(
        fields.addGenericError("rightOuttakeMotorWarning", this::getRightOuttakeMotorWarning));

    public final Supplier<GenericError> leftOuttakeMotorWarning = NetworkAlertsManager.addGenericError(
        fields.addGenericError("leftOuttakeMotorWarning", this::getLeftOuttakeMotorWarning));

    public final Supplier<GenericError> backMotorWarning = NetworkAlertsManager.addGenericError(
        fields.addGenericError("backMotorWarning", this::getBackMotorWarning));    

    public final Supplier<GenericError> rightOuttakeMotorConfigError = NetworkAlertsManager.addGenericError(
        fields.addGenericError("rightOuttakeMotorConfigError", this::getRightOuttakeMotorConfigError));

    public final Supplier<GenericError> leftOuttakeMotorConfigError = NetworkAlertsManager.addGenericError(
        fields.addGenericError("leftOuttakeMotorConfigError", this::getLeftOuttakeMotorConfigError));

    public final Supplier<GenericError> backMotorConfigError = NetworkAlertsManager.addGenericError(
        fields.addGenericError("backMotorConfigError", this::getBackMotorConfigError));
    

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    
    protected abstract boolean getIsCoralIn();
    
    protected abstract double getRightOuttakeMotorCurrent();
    
    protected abstract double getLeftOuttakeMotorCurrent();
    
    protected abstract double getBackMotorCurrent();

    
    protected abstract GenericError getRightOuttakeMotorError();
    
    protected abstract GenericError getLeftOuttakeMotorError();
    
    protected abstract GenericError getBackMotorError();

    protected abstract GenericError getRightOuttakeMotorWarning();
    
    protected abstract GenericError getLeftOuttakeMotorWarning();
    
    protected abstract GenericError getBackMotorWarning();

    protected abstract GenericError getRightOuttakeMotorConfigError();
    
    protected abstract GenericError getLeftOuttakeMotorConfigError();
    
    protected abstract GenericError getBackMotorConfigError();

    // Outputs:

    public abstract void setBreakMotor(boolean isBreak);

    public abstract void setRightOuttakeMotorVoltage(double voltage);

    public abstract void setLeftOuttakeMotorVoltage(double voltage);

    public abstract void setBackMotorVoltage(double voltage);
}
