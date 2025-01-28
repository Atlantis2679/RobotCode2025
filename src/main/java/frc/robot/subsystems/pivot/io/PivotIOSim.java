package frc.robot.subsystems.pivot.io;

import frc.lib.logfields.LogFieldsTable;

public class PivotIOSim extends PivotIO {
    
    public PivotIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }
    
    //inputs
    @Override
    protected double getSpeed() {
        return 0;
    }

    @Override
    protected double getPivotAngleDegrees() {
        return 0;
    }

    //outputs
    @Override
    public void setSpeed(double speed) {
    }
}
