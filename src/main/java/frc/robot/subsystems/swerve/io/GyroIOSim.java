package frc.robot.subsystems.swerve.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class GyroIOSim extends GyroIO{
    public GyroIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override 
    protected double getYawDegreesCW() {
        return 0;
    }

    @Override
    protected boolean getIsConnected() {
        return false;
    }
}