package frc.robot.subsystems.funnel.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.GenericError;
import frc.lib.networkalerts.NetworkAlertsManager;

public abstract class FunnelIO extends IOBase {
    public final BooleanSupplier isCoralIn = fields.addBoolean("isCoralIn", this::getIsCoralIn);
    public final DoubleSupplier motorCurrent = fields.addDouble("motorCurrent", this::getCurrent);
    public final Supplier<GenericError> motorError = NetworkAlertsManager.addGenericError(fields.addGenericError("motorError", this::getMotorError));
    public final Supplier<GenericError> motorWarning = NetworkAlertsManager.addGenericError(fields.addGenericError("motorWarning", this::getMotorWarning));
    public final Supplier<GenericError> motorConfigError = NetworkAlertsManager.addGenericError(fields.addGenericError("motorConfigError", this::getMotorConfigError));

    public FunnelIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    public abstract void setPercentageSpeed(double percentageSpeed);

    // Outputs:
    protected abstract boolean getIsCoralIn();

    protected abstract double getCurrent();

    protected abstract GenericError getMotorError();

    protected abstract GenericError getMotorWarning();

    protected abstract GenericError getMotorConfigError();
}
