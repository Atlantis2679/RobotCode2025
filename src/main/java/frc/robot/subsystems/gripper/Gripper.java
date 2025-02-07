package frc.robot.subsystems.gripper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.TuneableBuilder;
import frc.robot.Robot;
import frc.robot.subsystems.gripper.io.GripperIO;
import frc.robot.subsystems.gripper.io.GripperIOSim;
import frc.robot.subsystems.gripper.io.GripperIOSparkMax;

import static frc.robot.subsystems.gripper.GripperConstants.*;

public class Gripper extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final Debouncer isCoralInDebouncer = new Debouncer(DEBOUNCER_SECONDS);

    private double lastRightMotorVoltageDemand = 0;
    private double lastLeftMotorVoltageDemand = 0;
    private boolean lastDebouncerValue = false;

    private final GripperIO io = Robot.isReal() ? 
        new GripperIOSparkMax(fieldsTable) : 
        new GripperIOSim(fieldsTable);

    public Gripper() {
        fieldsTable.update();
    }

    @Override
    public void periodic() {
        fieldsTable.recordOutput("right motor real voltage", io.rightMotorVoltage.getAsDouble());
        fieldsTable.recordOutput("left motor real voltage", io.leftMotorVoltage.getAsDouble());
        fieldsTable.recordOutput("right motor voltage demand", lastRightMotorVoltageDemand);
        fieldsTable.recordOutput("left motor voltage demand", lastLeftMotorVoltageDemand);
        fieldsTable.recordOutput("last debauncer value", lastDebouncerValue);
        fieldsTable.recordOutput("isCoralIn", getIsCoralIn());
    }   

    public boolean getIsCoralIn() {
        return lastDebouncerValue = isCoralInDebouncer.calculate(io.isCoraIn.getAsBoolean());
    }

    public void setMotorsVoltage(double rightVoltage, double leftVoltage) {
        lastRightMotorVoltageDemand = rightVoltage;
        lastLeftMotorVoltageDemand = leftVoltage;
        io.setRightMotorVoltage(MathUtil.clamp(rightVoltage, -RIGHT_MOTOR_MAX_VOLTAGE, RIGHT_MOTOR_MAX_VOLTAGE));
        io.setLeftMotorVoltage(MathUtil.clamp(leftVoltage, -LEFT_MOTOR_MAX_VOLTAGE, LEFT_MOTOR_MAX_VOLTAGE));
    }

    public void stop() {
        lastRightMotorVoltageDemand = 0;
        lastLeftMotorVoltageDemand = 0;
        io.setRightMotorVoltage(0);
        io.setLeftMotorVoltage(0);
    }
    //     public void initTuneable(TuneableBuilder builder) {
    //     builder.addChild("Pivot PID", pivotPidController);
    // }
}
