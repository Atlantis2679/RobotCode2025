package frc.robot.subsystems.gripper.io;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.NetworkAlertsManager;
import frc.lib.networkalerts.NetworkPeriodicAlert;

public abstract class GripperIO extends IOBase {
    public final BooleanSupplier isCoralIn = fields.addBoolean("isCoralIn", this::getIsCoralIn);
    public final DoubleSupplier rightOuttakeMotorCurrent = fields.addDouble("rightOuttakeMotorCurrect",
            this::getRightOuttakeMotorCurrent);
    public final DoubleSupplier leftOuttakeMotorCurrent = fields.addDouble("leftOuttakeMotorCurrent",
            this::getLeftOuttakeMotorCurrent);
    public final DoubleSupplier backMotorCurrent = fields.addDouble("backMotorCurrent", this::getBackMotorCurrent);
    public final NetworkPeriodicAlert[] rightOuttakeMotorAlerts = NetworkAlertsManager.addNetworkPeriodicAlertsArray(
        fields.addNetworkPeriodicAlertsArray("rightOuttakeMotorAlerts", getRightOuttakeMotorAlerts()));
    public final NetworkPeriodicAlert[] leftOuttakeMotorAlerts = NetworkAlertsManager.addNetworkPeriodicAlertsArray(
        fields.addNetworkPeriodicAlertsArray("leftOuttakeMotorAlerts", getLeftOuttakeMotorAlerts()));
    public final NetworkPeriodicAlert[] backMotorAlerts = NetworkAlertsManager.addNetworkPeriodicAlertsArray(
        fields.addNetworkPeriodicAlertsArray("backMotorAlerts", getBackMotorAlerts()));

    public GripperIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:
    
    protected abstract boolean getIsCoralIn();
    
    protected abstract double getRightOuttakeMotorCurrent();
    
    protected abstract double getLeftOuttakeMotorCurrent();
    
    protected abstract double getBackMotorCurrent();

    protected abstract Map<String, NetworkPeriodicAlert> getRightOuttakeMotorAlerts();

    protected abstract Map<String, NetworkPeriodicAlert> getLeftOuttakeMotorAlerts();

    protected abstract Map<String, NetworkPeriodicAlert> getBackMotorAlerts();
    
    // Outputs:

    public abstract void setBreakMotor(boolean isBreak);

    public abstract void setRightOuttakeMotorVoltage(double voltage);

    public abstract void setLeftOuttakeMotorVoltage(double voltage);

    public abstract void setBackMotorVoltage(double voltage);
}
