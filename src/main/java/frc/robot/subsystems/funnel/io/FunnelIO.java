package frc.robot.subsystems.funnel.io;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.NetworkAlertsManager;
import frc.lib.networkalerts.NetworkPeriodicAlert;

public abstract class FunnelIO extends IOBase {
    public final BooleanSupplier isCoralIn = fields.addBoolean("isCoralIn", this::getIsCoralIn);
    public final DoubleSupplier motorCurrent = fields.addDouble("motorCurrent", this::getCurrent);
    public final NetworkPeriodicAlert[] motoraAlerts = NetworkAlertsManager.addNetworkPeriodicAlertsArray(
        fields.addNetworkPeriodicAlertsArray("motorAlerts", getMotorAlerts()));
    
    public FunnelIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    public abstract void setPercentageSpeed(double percentageSpeed);

    // Outputs:
    protected abstract boolean getIsCoralIn();

    protected abstract double getCurrent();

    protected abstract Map<String, NetworkPeriodicAlert> getMotorAlerts();
}
