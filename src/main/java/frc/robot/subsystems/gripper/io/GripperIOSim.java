package frc.robot.subsystems.gripper.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

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
    public void setRightMotorVoltage(double voltage) {
    }

    @Override
    public void setLeftMotorVoltage(double voltage) {
    }

    @Override
    public void setBackMotorVoltage(double voltage) {
    }

    @Override
    protected double getRightMotorCurrent() {
        return 0;
    }

    @Override
    protected double getLeftMotorCurrent() {
        return 0;
    }

    @Override
    protected double getBackMotorCurrent() {
        return 0;
    }
}
