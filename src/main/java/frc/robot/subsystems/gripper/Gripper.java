package frc.robot.subsystems.gripper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.Robot;
import frc.robot.subsystems.gripper.io.GripperIO;
import frc.robot.subsystems.gripper.io.GripperIOSim;
import frc.robot.subsystems.gripper.io.GripperIOSparkMax;

import static frc.robot.subsystems.gripper.GripperConstants.*;

public class Gripper extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final Debouncer isCoralInDebouncer = new Debouncer(DEBOUNCER_SECONDS, DebounceType.kBoth);

    private final GripperIO io = Robot.isReal() ? new GripperIOSparkMax(fieldsTable) : new GripperIOSim(fieldsTable);

    public Gripper() {}

    @Override
    public void periodic() {
        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        SmartDashboard.putBoolean("CoralInGripper", getIsCoralIn());
        fieldsTable.recordOutput("isCoralIn", getIsCoralIn());
    }

    public boolean getIsCoralIn() {
        return isCoralInDebouncer.calculate(io.isCoralIn.getAsBoolean());
    }

    public void setBreakMotors(){
        io.setBreakMotor(true);
    }

    public void setCoastMotors(){
        io.setBreakMotor(false);
    }

    public void setMotorsVoltages(double rightOuttakeVoltage, double leftOuttakeVoltage, double backMotorVoltage) {
        fieldsTable.recordOutput("Right outtake voltage", rightOuttakeVoltage);
        fieldsTable.recordOutput("Left outtake voltage", leftOuttakeVoltage);
        fieldsTable.recordOutput("Back voltage", backMotorVoltage);

        io.setRightOuttakeMotorVoltage(
                MathUtil.clamp(rightOuttakeVoltage, -OUTTAKE_MOTORS_MAX_VOLTAGE, OUTTAKE_MOTORS_MAX_VOLTAGE));
        io.setLeftOuttakeMotorVoltage(
                MathUtil.clamp(leftOuttakeVoltage, -OUTTAKE_MOTORS_MAX_VOLTAGE, OUTTAKE_MOTORS_MAX_VOLTAGE));
        io.setBackMotorVoltage(
            MathUtil.clamp(backMotorVoltage, -BACK_MOTOR_MAX_VOLTAGE, BACK_MOTOR_MAX_VOLTAGE));
    }

    public void stop() {
        fieldsTable.recordOutput("Right outtake voltage", 0.0);
        fieldsTable.recordOutput("Left outtake voltage", 0.0);
        fieldsTable.recordOutput("Back voltage", 0.0);

        io.setRightOuttakeMotorVoltage(0);
        io.setLeftOuttakeMotorVoltage(0);
        io.setBackMotorVoltage(0);
    }
}
