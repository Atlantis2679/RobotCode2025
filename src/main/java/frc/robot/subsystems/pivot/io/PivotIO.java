package frc.robot.subsystems.pivot.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.GenericError;
import frc.lib.networkalerts.NetworkAlertsManager;

public abstract class PivotIO extends IOBase {
    public final DoubleSupplier motorCurrent = fields.addDouble("motor current", this::getMotorCurrent);
    public final DoubleSupplier angle = fields.addDouble("angle", this::getPivotAngleDegrees);
    public final BooleanSupplier isEncoderConnected = fields.addBoolean("isEncoderConnected", this::getIsEncoderConnected);
    public final Supplier<GenericError> motorError = NetworkAlertsManager.addGenericError(fields.addGenericError("motorError", this::getMotorError));
    public final Supplier<GenericError> motorWarning = NetworkAlertsManager.addGenericError(fields.addGenericError("motorWarning", this::getMotorWarning));
    public final Supplier<GenericError> motorConfigError = NetworkAlertsManager.addGenericError(fields.addGenericError("motorConfigError", this::getMotorConfigError));

    public PivotIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    protected abstract double getMotorCurrent();

    protected abstract double getPivotAngleDegrees();
    
    protected abstract boolean getIsEncoderConnected();

    protected abstract GenericError getMotorError();

    protected abstract GenericError getMotorWarning();

    protected abstract GenericError getMotorConfigError();

    // Outputs:
    public abstract void setVoltage(double voltage);
}