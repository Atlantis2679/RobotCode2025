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

    public void setOuttakeMotorsVoltage(double rightOutTakeVoltage, double leftOutTakeVoltage) {
        io.setRightOutTakeMotorVoltage(MathUtil.clamp(rightOutTakeVoltage, -OUTTAKE_MOTORS_MAX_VOLTAGE, OUTTAKE_MOTORS_MAX_VOLTAGE));
        io.setLeftOutTakeMotorVoltage(MathUtil.clamp(leftOutTakeVoltage, -OUTTAKE_MOTORS_MAX_VOLTAGE, OUTTAKE_MOTORS_MAX_VOLTAGE));
    }

    public void setBackMotorVoltage(double backMotorVoltage) {
        io.setBackMotorVoltage(MathUtil.clamp(backMotorVoltage, -BACK_MOTOR_MAX_VOLTAGE, BACK_MOTOR_MAX_VOLTAGE));
    }

    public void stop() {
        setOuttakeMotorsVoltage(0, 0);
        setBackMotorVoltage(0);
    }
}
