package frc.robot.subsystems.gripper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.Robot;
import frc.robot.subsystems.gripper.io.GripperIO;
import frc.robot.subsystems.gripper.io.GripperIOSim;
import frc.robot.subsystems.gripper.io.GripperIOSparkMax;

import static frc.robot.subsystems.gripper.GripperConstants.*;

public class Gripper extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final GripperIO io = Robot.isReal() ? 
        new GripperIOSparkMax(fieldsTable.getSubTable("io")) : 
        new GripperIOSim(fieldsTable.getSubTable("io"));

    public Gripper() {
    }

    public boolean getIsCoralIn() {
        return io.isCoraIn.getAsBoolean();
    }

    public double getRightOutTakeMotorSpeedRPM() {
        return io.rightOutTakeMotorSpeedRPM.getAsDouble();
    }

    public double getLeftOutTakeMotorSpeedRPM() {
        return io.leftOutTakeMotorSpeedRPM.getAsDouble();
    }

    public double getBackMotorSpeedRPM() {
        return io.backMotorSpeedRPM.getAsDouble();
    }

    public void setOuttakeMotorsVoltage(double rightOutTakeVoltage, double leftOutTakeVoltage) {
        io.setRightOutTakeMotorVoltage(MathUtil.clamp(rightOutTakeVoltage, -OUTTAKE_MOTORS_MAX_VOLTAGE, OUTTAKE_MOTORS_MAX_VOLTAGE));
        io.setLeftOutTakeMotorVoltage(MathUtil.clamp(leftOutTakeVoltage, -OUTTAKE_MOTORS_MAX_VOLTAGE, OUTTAKE_MOTORS_MAX_VOLTAGE));
    }

    public void setOuttakeMotorsPrecentageSpeed(double rightOutTakePrecentageSpeed, double leftOutTakePrecentageSpeed) {
        io.setRightOutTakeMotorVoltage(MathUtil.clamp(rightOutTakePrecentageSpeed, -1, 1));
        io.setLeftOutTakeMotorVoltage(MathUtil.clamp(leftOutTakePrecentageSpeed, -1, 1));
    }

    public void setBackMotorVoltage(double intakeOutTakeVoltage) {
        io.setBackMotorVoltage(MathUtil.clamp(intakeOutTakeVoltage, -BACK_MOTOR_MAX_VOLTAGE, BACK_MOTOR_MAX_VOLTAGE));
    }

    public void setBackMotorPrecentageSpeed(double intakeOutTakePrecentageSpeed) {
        io.setBackMotorVoltage(MathUtil.clamp(intakeOutTakePrecentageSpeed, -1, 1));
    }
}
