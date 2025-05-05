package frc.robot.subsystems.gripper.io;

import java.util.Map;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkBase.Faults;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.NetworkPeriodicAlert;
import frc.robot.utils.AlertsFactory;

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

    @Override
    protected Map<String, NetworkPeriodicAlert> getRightOuttakeMotorAlerts() {
        return AlertsFactory.revMotor(
            () -> REVLibError.kOk, () -> new Warnings(0), () -> new Faults(0), "Gripper", "rightOuttakeMotor");   
    }

    @Override
    protected Map<String, NetworkPeriodicAlert> getLeftOuttakeMotorAlerts() {
        return AlertsFactory.revMotor(
            () -> REVLibError.kOk, () -> new Warnings(0), () -> new Faults(0), "Gripper", "leftOuttakeMotor");
    }

    @Override
    protected Map<String, NetworkPeriodicAlert> getBackMotorAlerts() {
        return AlertsFactory.revMotor(
            () -> REVLibError.kOk, () -> new Warnings(0), () -> new Faults(0), "Gripper", "backMotor");
    }
}
