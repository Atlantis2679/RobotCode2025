package frc.robot.subsystems.funnel.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

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
}