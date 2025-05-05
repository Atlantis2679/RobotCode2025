package frc.robot.subsystems.gripper.io;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkBase.Faults;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.NetworkPeriodicAlert;

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
        Map<String, NetworkPeriodicAlert> alerts = new HashMap<String, NetworkPeriodicAlert>();
        
    }

    @Override
    protected Map<String, NetworkPeriodicAlert> getLeftOuttakeMotorAlerts() {
        Map<String, NetworkPeriodicAlert> alerts = new HashMap<String, NetworkPeriodicAlert>();

    }

    @Override
    protected Map<String, NetworkPeriodicAlert> getBackMotorAlerts() {
        Map<String, NetworkPeriodicAlert> alerts = new HashMap<String, NetworkPeriodicAlert>();

    }
}
