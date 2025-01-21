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

    private final GripperIO io = Robot.isReal() ? 
        new GripperIOSparkMax(fieldsTable.getSubTable("io")) : 
        new GripperIOSim(fieldsTable.getSubTable("io"));

    public Gripper() {
    }

    public boolean getIsCoralIn() {
        return isCoralInDebouncer.calculate(io.isCoraIn.getAsBoolean());
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

    public void setOutTakeMotorsPercentSpeed(double rightOutTakePercentSpeed, double leftOutTakePercentSpeed) {
        io.setRightOutTakeMotorVoltage(MathUtil.clamp(rightOutTakePercentSpeed, -1, 1));
        io.setLeftOutTakeMotorVoltage(MathUtil.clamp(leftOutTakePercentSpeed, -1, 1));
    }

    public void setBackMotorVoltage(double backMotorVoltage) {
        io.setBackMotorVoltage(MathUtil.clamp(backMotorVoltage, -BACK_MOTOR_MAX_VOLTAGE, BACK_MOTOR_MAX_VOLTAGE));
    }

    public void setBackMotorPercentSpeed(double backOutTakePercentSpeed) {
        io.setBackMotorVoltage(MathUtil.clamp(backOutTakePercentSpeed, -1, 1));
    }

    public boolean isOutTakeMotorsAtSpeed(double rightOutTakeMotorSpeedRPM, double leftOutTakeMotorSpeedRPM) {
        return getRightOutTakeMotorSpeedRPM() >= rightOutTakeMotorSpeedRPM &&
            getLeftOutTakeMotorSpeedRPM() >= leftOutTakeMotorSpeedRPM;
    }

    public boolean isBackMotorAtSpeed(double backMotorSpeedRPM) {
        return getBackMotorSpeedRPM() >= backMotorSpeedRPM;
    }

    public void stop() {
        setOuttakeMotorsVoltage(0, 0);
        setBackMotorVoltage(0);
    }
}
