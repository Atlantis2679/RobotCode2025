package frc.robot.subsystems.gripper.io;

import frc.lib.logfields.LogFieldsTable;

public class GripperIOSim extends GripperIO {
    public GripperIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:

    @Override
    protected boolean getIsCoralIn() {
        return false;
    }

    // Outputs:

    @Override
    public void setBreakMotor(boolean isBreak) {
    }

    @Override
    public void setRightOuttakeMotorVoltage(double voltage) {
    }

    @Override
    public void setLeftOuttakeMotorVoltage(double voltage) {
    }

    @Override
    public void setBackMotorVoltage(double voltage) {
    }

    @Override
    protected double getRightOuttakeMotorCurrent() {
        return 0;
    }

    @Override
    protected double getLeftOuttakeMotorCurrent() {
        return 0;
    }

    @Override
    protected double getBackMotorCurrent() {
        return 0;
    }
}
