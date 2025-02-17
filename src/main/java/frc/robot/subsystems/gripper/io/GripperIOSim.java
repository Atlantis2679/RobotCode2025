package frc.robot.subsystems.gripper.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.subsystems.gripper.GripperConstants.*;

public class GripperIOSim extends GripperIO {
    private final FlywheelSim rightMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), GripperSim.MOTORS_MOMENT_OF_INERTIA,
        GripperSim.MOTORS_GEAR_RATIO), DCMotor.getNeo550(1));
    
    private final FlywheelSim leftMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), GripperSim.MOTORS_MOMENT_OF_INERTIA,
        GripperSim.MOTORS_GEAR_RATIO), DCMotor.getNeo550(1));
    
    public GripperIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected void periodicBeforeFields() {
        rightMotor.update(0.02);
        leftMotor.update(0.02);
    }

    // Inputs:

    @Override
    protected boolean getIsCoralIn() {
        return false;
    }
  
    @Override
    protected int getRightOuttakeMotorStatusValue() {
        return 0;
    }

    @Override
    protected int getLeftOuttakeMotorStatusValue() {
        return 0;
    }
    
    @Override
    protected double getLeftMotorVoltage() {
        return leftMotor.getInputVoltage();
    }

    @Override
    protected double getRightMotorVoltage() {
        return rightMotor.getInputVoltage();
    }

    // Outputs:

    @Override
    public void setRightMotorVoltage(double voltage) {
        rightMotor.setInputVoltage(voltage);
    }

    @Override
    public void setLeftMotorVoltage(double voltage) {
        leftMotor.setInputVoltage(voltage);
    }
}
