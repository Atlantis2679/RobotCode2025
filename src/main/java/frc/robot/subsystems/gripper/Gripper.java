package frc.robot.subsystems.gripper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
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
    private double lastBackMotorVoltageDeamand = 0;

    private final GripperIO io = Robot.isReal() ? 
        new GripperIOSparkMax(fieldsTable) : 
        new GripperIOSim(fieldsTable);

    public Gripper() {
    }

    @Override
    public void periodic() {
        // voltage deamand isn't clamped for calibrations and testing perposes!
        fieldsTable.recordOutput("Right motor voltage demand", lastRightMotorVoltageDemand);
        fieldsTable.recordOutput("Left motor voltage demand", lastLeftMotorVoltageDemand);
        fieldsTable.recordOutput("Back motor voltage demand", lastBackMotorVoltageDeamand);
    }

    public boolean getIsCoralIn() {
        return isCoralInDebouncer.calculate(io.isCoraIn.getAsBoolean());
    }

    public void setOuttakeMotorsVoltage(double rightOuttakeVoltage, double leftOuttakeVoltage) {
        lastRightMotorVoltageDemand = rightOuttakeVoltage;
        lastLeftMotorVoltageDemand = leftOuttakeVoltage;
        io.setRightOuttakeMotorVoltage(MathUtil.clamp(rightOuttakeVoltage, -OUTTAKE_MOTORS_MAX_VOLTAGE, OUTTAKE_MOTORS_MAX_VOLTAGE));
        io.setLeftOuttakeMotorVoltage(MathUtil.clamp(leftOuttakeVoltage, -OUTTAKE_MOTORS_MAX_VOLTAGE, OUTTAKE_MOTORS_MAX_VOLTAGE));
    }

    public void setBackMotorVoltage(double backMotorVoltage) {
        lastBackMotorVoltageDeamand = backMotorVoltage;
        io.setBackMotorVoltage(MathUtil.clamp(backMotorVoltage, -BACK_MOTOR_MAX_VOLTAGE, BACK_MOTOR_MAX_VOLTAGE));
    }

    public void setMotorsVoltage(double rightOuttakeVoltage, double leftOuttakeVoltage, double backMotorVoltage) {
        lastRightMotorVoltageDemand = rightOuttakeVoltage;
        lastLeftMotorVoltageDemand = leftOuttakeVoltage;
        lastBackMotorVoltageDeamand = backMotorVoltage;
        setOuttakeMotorsVoltage(rightOuttakeVoltage, leftOuttakeVoltage);
        setBackMotorVoltage(backMotorVoltage);
    }

    public void stop() {
        lastRightMotorVoltageDemand = 0;
        lastLeftMotorVoltageDemand = 0;
        lastBackMotorVoltageDeamand = 0;
        io.setRightOuttakeMotorVoltage(0);
        io.setLeftOuttakeMotorVoltage(0);
        io.setBackMotorVoltage(0);
    }
}
