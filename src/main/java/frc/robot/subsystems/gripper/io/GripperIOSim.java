package frc.robot.subsystems.gripper.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.subsystems.gripper.GripperConstants.*;

public class GripperIOSim extends GripperIO {
    private double lastRightOuttakeMotorDesiredVoltage = 0;
    private double lastLeftOuttakeMotorDesiredVoltage = 0;
    private double lastBackMotorDesiredVoltage = 0;

    private final FlywheelSim rightOuttakeMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), GripperSim.OUTTAKE_MOTORS_MOMENT_OF_INERTIA,
        GripperSim.OUTTAKE_MOTORS_GEAR_RATIO), DCMotor.getNeo550(1));
    
    private final FlywheelSim leftOuttakeMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), GripperSim.OUTTAKE_MOTORS_MOMENT_OF_INERTIA,
        GripperSim.OUTTAKE_MOTORS_GEAR_RATIO), DCMotor.getNeo550(1));
    
    private final FlywheelSim backMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), GripperSim.BACK_MOTOR_MOMENT_OF_INERTIA,
        GripperSim.BACK_MOTOR_GEAR_RATIO), DCMotor.getNEO(1));
    
    public GripperIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected void periodicBeforeFields() {
        rightOuttakeMotor.update(0.02);
        leftOuttakeMotor.update(0.02);
        backMotor.update(0.02);
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
        rightOuttakeMotor.setInputVoltage(voltage);
    }

    @Override
    public void setLeftOuttakeMotorVoltage(double voltage) {
        lastLeftOuttakeMotorDesiredVoltage = voltage;
        leftOuttakeMotor.setInputVoltage(voltage);
    }

    @Override
    public void setBackMotorVoltage(double voltage) {
        lastBackMotorDesiredVoltage = voltage;
        backMotor.setInputVoltage(voltage);
    }

    @Override
    protected double getRightOuttakeMotorVoltage() {
        return lastRightOuttakeMotorDesiredVoltage;
    }

    @Override
    protected double getRightOuttakeMotorCurrent() {
        return rightOuttakeMotor.getCurrentDrawAmps();
    }

    @Override
    protected double getLeftOuttakeMotorVoltage() {
        return lastLeftOuttakeMotorDesiredVoltage;
    }

    @Override
    protected double getLeftOuttakeMotorCurrent() {
        return leftOuttakeMotor.getCurrentDrawAmps();
    }

    @Override
    protected double getBackMotorVoltage() {
        return lastBackMotorDesiredVoltage;
    }

    @Override
    protected double getBackMotorCurrent() {
        return backMotor.getCurrentDrawAmps();
    }
}
