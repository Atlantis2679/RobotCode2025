package frc.robot.subsystems.pivot.io;

import static frc.robot.subsystems.pivot.PivotConstants.Sim.*;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import static frc.robot.subsystems.pivot.PivotConstants.INITIAL_OFFSET;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.logfields.LogFieldsTable;

public class PivotIOSim extends PivotIO {
    private final SingleJointedArmSim pivotMotor = new SingleJointedArmSim(
        DCMotor.getNEO(1),
        JOINT_GEAR_RATIO,
        JKG_METERS_SQUARED,
        ARM_LENGTH,
        Math.toRadians(TURNING_MIN_DEGREES),
        Math.toRadians(TURNING_MAX_DEGREES),
        true,
        INITIAL_OFFSET);
    
    private double lastVoltage = 0;
    
    public PivotIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected void periodicBeforeFields() {
        pivotMotor.update(0.02);
    }

    // Outputs:    
    @Override
    protected double getMotorCurrent() {
        return pivotMotor.getCurrentDrawAmps();
    }

    @Override
    protected double getPivotAngleDegrees() {
        return Math.toDegrees(pivotMotor.getAngleRads());
    }

    // Inputs:
    @Override
    public void setVoltage(double voltage) {
        lastVoltage = voltage;
        pivotMotor.setInputVoltage(voltage);
    }

    @Override
    protected double getMotorVoltage() {
        return lastVoltage;
    }

    @Override
    protected REVLibError getMotorConfigError() {
        return REVLibError.fromInt(0);
    }

    @Override
    protected Faults getMotorFaults() {
        return new Faults(0);
    }

    @Override
    protected Warnings getMotorWarnings() {
        return new Warnings(0);
    }
}