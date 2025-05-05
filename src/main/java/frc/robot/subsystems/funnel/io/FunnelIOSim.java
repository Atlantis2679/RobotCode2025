package frc.robot.subsystems.funnel.io;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.NetworkPeriodicAlert;
import frc.robot.utils.AlertsFactory;

public class FunnelIOSim extends FunnelIO {
    public FunnelIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    public void setPercentageSpeed(double percentageSpeed) {
    }

    @Override
    protected boolean getIsCoralIn() {
        return false;
    }

    @Override
    protected double getCurrent() {
        return 0;
    }

    @Override
    protected Map<String, NetworkPeriodicAlert> getMotorAlerts() {
        Map<String, NetworkPeriodicAlert> alerts = new HashMap<String, NetworkPeriodicAlert>();
        alerts.put("configError", AlertsFactory.revError(REVLibError.kOk, "Funnel", "motor"));
        alerts.put("error", AlertsFactory.sparkMaxError(() -> new Faults(0), "Funnel", "motor"));
        alerts.put("warning", AlertsFactory.sparkMaxWarning(() -> new Warnings(0), "Funnel", "motor"));
        return alerts;
    }
}