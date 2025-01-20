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
    
    private final FlywheelSim intakeOutTakeMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), GripperSim.INTAKE_OUTTAKE_MOTOR_MOMENT_OF_INERTIA,
        GripperSim.INTAKE_OUTTAKE_MOTOR_GEAR_RATIO), DCMotor.getNEO(1));
    
    public GripperIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected void periodicBeforeFields() {
        rightOutTakeMotor.update(0.02);
        leftOutTakeMotor.update(0.02);
        intakeOutTakeMotor.update(0.02);
    }

    // Inputs:

    @Override
    protected boolean getIsCoralIn() {
        return false;
    }

    @Override
    protected double getRightOutTakeMotorSpeedRPM() {
        return rightOutTakeMotor.getAngularVelocityRPM();
    }

    @Override
    protected double getLeftOutTakeMotorSpeedRPM() {
        return leftOutTakeMotor.getAngularVelocityRPM();
    }

    @Override
    protected double getIntakeOutTakeMotorSpeedRPM() {
        return intakeOutTakeMotor.getAngularVelocityRPM();
    }

    // Outputs:

    @Override
    public void setVoltageRightOutTake(double voltage) {
        rightOutTakeMotor.setInputVoltage(voltage);
    }

    @Override
    public void setPrecentageSpeedRightOutTake(double precentageSpeed) {
        rightOutTakeMotor.setInputVoltage(precentageSpeed * OUTTAKE_MOTORS_MAX_VOLTAGE);
    }

    @Override
    public void setVoltageLeftOutTake(double voltage) {
        leftOutTakeMotor.setInputVoltage(voltage);
    }

    @Override
    public void setPrecentageSpeedLeftOutTake(double precentageSpeed) {
        leftOutTakeMotor.setInputVoltage(precentageSpeed * OUTTAKE_MOTORS_MAX_VOLTAGE);
    }

    @Override
    public void setVoltageIntakeOutTake(double voltage) {
        intakeOutTakeMotor.setInputVoltage(voltage);
    }

    @Override
    public void setPrecentageSpeedIntakeOutTake(double precentageSpeed) {
        intakeOutTakeMotor.setInputVoltage(precentageSpeed * INTAKE_OUTTAKE_MOTOR_MAX_VOLTAGE);
    }
}
