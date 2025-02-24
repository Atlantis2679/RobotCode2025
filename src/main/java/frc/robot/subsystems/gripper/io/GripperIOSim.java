package frc.robot.subsystems.gripper.io;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import frc.lib.logfields.LogFieldsTable;

public class GripperIOSim extends GripperIO {
    private double lastRightOuttakeMotorDesiredVoltage = 0;
    private double lastLeftOuttakeMotorDesiredVoltage = 0;
    private double lastBackMotorDesiredVoltage = 0;
    
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
    public void setRightOuttakeMotorVoltage(double voltage) {
        lastRightOuttakeMotorDesiredVoltage = voltage;
    }

    @Override
    public void setLeftOuttakeMotorVoltage(double voltage) {
        lastLeftOuttakeMotorDesiredVoltage = voltage;
    }

    @Override
    public void setBackMotorVoltage(double voltage) {
        lastBackMotorDesiredVoltage = voltage;
    }

    @Override
    protected double getRightOuttakeMotorVoltage() {
        return lastRightOuttakeMotorDesiredVoltage;
    }

    @Override
    protected double getRightOuttakeMotorCurrent() {
        return 0;
    }

    @Override
    protected double getLeftOuttakeMotorVoltage() {
        return lastLeftOuttakeMotorDesiredVoltage;
    }

    @Override
    protected double getLeftOuttakeMotorCurrent() {
        return 0;
    }

    @Override
    protected double getBackMotorVoltage() {
        return lastBackMotorDesiredVoltage;
    }

    @Override
    protected double getBackMotorCurrent() {
        return 0;
    }

    @Override
    protected REVLibError getRightOuttakeMotorConfigError() {
        return REVLibError.fromInt(0);
    }

    @Override
    protected REVLibError getLeftOuttakeMotorConfigError() {
        return REVLibError.fromInt(0);
    }

    @Override
    protected REVLibError getBackMotorConfigError() {
        return REVLibError.fromInt(0);
    }

    @Override
    protected Faults getRightOuttakeMotorFaults() {
        return new Faults(0);
    }

    @Override
    protected Faults getLeftOuttakeMotorFaults() {
        return new Faults(0);
    }

    @Override
    protected Faults getBackMotorFaults() {
        return new Faults(0);
    }

    @Override
    protected Warnings getRightOuttakeMotorWarnings() {
        return new Warnings(0);
    }

    @Override
    protected Warnings getLeftOuttakeMotorWarnings() {
        return new Warnings(0);
    }

    @Override
    protected Warnings getBackMotorWarnings() {
        return new Warnings(0);
    }
}
