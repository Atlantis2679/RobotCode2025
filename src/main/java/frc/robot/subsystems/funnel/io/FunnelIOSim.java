package frc.robot.subsystems.funnel.io;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import frc.lib.logfields.LogFieldsTable;

public class FunnelIOSim extends FunnelIO {

    public FunnelIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    public void setVoltage(double voltageDemand) {
    }

    @Override
    public void setPercentageSpeed(double percentageSpeed) {
    }

    @Override
    protected boolean getIsCoralIn() {
        return false;
    }

    @Override
    protected double getVoltage() {
        return 0;
    }

    @Override
    protected double getCurrent() {
        return 0;
    }

    @Override
    protected REVLibError getConfigError() {
        return REVLibError.fromInt(0);
    }

    @Override
    protected Faults getSparkFaults() {
        return new Faults(0);
    }

    @Override
    protected Warnings getSparkWarnings() {
        return new Warnings(0);
    }
}