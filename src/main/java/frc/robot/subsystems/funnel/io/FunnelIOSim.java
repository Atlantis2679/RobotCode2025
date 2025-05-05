package frc.robot.subsystems.funnel.io;

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
        return AlertsFactory.revMotor(
            REVLibError.kOk, () -> new Warnings(0), () -> new Faults(0), "Funnel", "Motor");
    }
}