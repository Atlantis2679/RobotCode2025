package frc.robot.subsystems.gripper.io;

import frc.lib.logfields.LogFieldsTable;

public class GripperIOSim extends GripperIO {
    
    public GripperIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected boolean getIsCoralIn() {
        return false;
    }

    @Override
    public void setVoltageRightOutTake(double voltage) {
    }

    @Override
    public void setPrecentageSpeedRightOutTake(double precentageSpeed) {
    }

    @Override
    public void setVoltageLeftOutTake(double voltage) {
    }

    @Override
    public void setPrecentageSpeedLeftOutTake(double precentageSpeed) {
    }

    @Override
    public void setVoltageIntakeOutTake(double voltage) {
    }

    @Override
    public void setPrecentageSpeedIntakeOutTake(double precentageSpeed) {
    }

    @Override
    protected double getRightOutTakeMotorSpeedRPM() {
        return 0;
    }

    @Override
    protected double getLeftOutTakeMotorSpeedRPM() {
        return 0;
    }

    @Override
    protected double getIntakeOutTakeMotorSpeedRPM() {
        return 0;
    }
}
