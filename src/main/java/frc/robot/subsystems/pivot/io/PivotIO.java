package frc.robot.subsystems.pivot.io;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.NetworkAlertsManager;
import frc.lib.networkalerts.NetworkPeriodicAlert;

public abstract class PivotIO extends IOBase {
    public final DoubleSupplier motorCurrent = fields.addDouble("motor current", this::getMotorCurrent);
    public final DoubleSupplier angle = fields.addDouble("angle", this::getPivotAngleDegrees);
    public final BooleanSupplier isEncoderConnected = fields.addBoolean("isEncoderConnected", this::getIsEncoderConnected);
    public final NetworkPeriodicAlert[] motorAlerts = NetworkAlertsManager.addNetworkPeriodicAlertsArray(
        fields.addNetworkPeriodicAlertsArray("motorAlerts", getMotorAlerts()));

    public PivotIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    protected abstract double getMotorCurrent();

    protected abstract double getPivotAngleDegrees();
    
    protected abstract boolean getIsEncoderConnected();

    protected abstract Map<String, NetworkPeriodicAlert> getMotorAlerts();

    // Outputs:
    public abstract void setVoltage(double voltage);
}