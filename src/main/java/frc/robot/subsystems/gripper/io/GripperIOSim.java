package frc.robot.subsystems.gripper.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.subsystems.gripper.GripperConstants.*;

public class GripperIOSim extends GripperIO {
    private final FlywheelSim rightOutTakeMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), GripperSim.OUTTAKE_MOTORS_MOMENT_OF_INERTIA,
        GripperSim.OUTTAKE_MOTORS_GEAR_RATIO), DCMotor.getNeo550(1));
    
    private final FlywheelSim leftOutTakeMotor = new FlywheelSim(
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
        rightOutTakeMotor.update(0.02);
        leftOutTakeMotor.update(0.02);
        backMotor.update(0.02);
    }

    // Inputs:

    @Override
    protected boolean getIsCoralIn() {
        return false;
    }

    // Outputs:

    @Override
    public void setRightOutTakeMotorVoltage(double voltage) {
        rightOutTakeMotor.setInputVoltage(voltage);
    }

    @Override
    public void setRightOutTakeMotorPrecentageSpeed(double precentageSpeed) {
        rightOutTakeMotor.setInputVoltage(precentageSpeed * OUTTAKE_MOTORS_MAX_VOLTAGE);
    }

    @Override
    public void setLeftOutTakeMotorVoltage(double voltage) {
        leftOutTakeMotor.setInputVoltage(voltage);
    }

    @Override
    public void setLeftOutTakeMotorPrecentageSpeed(double precentageSpeed) {
        leftOutTakeMotor.setInputVoltage(precentageSpeed * OUTTAKE_MOTORS_MAX_VOLTAGE);
    }

    @Override
    public void setBackMotorVoltage(double voltage) {
        backMotor.setInputVoltage(voltage);
    }

    @Override
    public void setBackMotorPrecentageSpeed(double precentageSpeed) {
        backMotor.setInputVoltage(precentageSpeed * BACK_MOTOR_MAX_VOLTAGE);
    }
}
