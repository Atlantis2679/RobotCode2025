package frc.robot.subsystems.funnel.io;

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
    protected int getMotorStatusValue() {
        return 0;
    }
}