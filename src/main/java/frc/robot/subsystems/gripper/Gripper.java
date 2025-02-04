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
        new GripperIOSparkMax(fieldsTable) : 
        new GripperIOSim(fieldsTable);

    public Gripper() {
    }

    public boolean getIsCoralIn() {
        return isCoralInDebouncer.calculate(io.isCoraIn.getAsBoolean());
    }

    public void setMotorsVoltage(double rightOuttakeVoltage, double leftOuttakeVoltage) {
        io.setRightMotorVoltage(MathUtil.clamp(rightOuttakeVoltage, -RIGHT_MOTOR_MAX_VOLTAGE, RIGHT_MOTOR_MAX_VOLTAGE));
        io.setLeftMotorVoltage(MathUtil.clamp(leftOuttakeVoltage, -LEFT_MOTOR_MAX_VOLTAGE, LEFT_MOTOR_MAX_VOLTAGE));
    }

    public void stop() {
        io.setRightMotorVoltage(0);
        io.setLeftMotorVoltage(0);
    }
}
