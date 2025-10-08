package frc.robot.subsystems.gripper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team2679.atlantiskit.logfields.LogFieldsTable;
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

    public void setMotorsVoltages(double rightVoltage, double leftVoltage, double backVoltage) {
        fieldsTable.recordOutput("Right motor demand voltage", rightVoltage);
        fieldsTable.recordOutput("Left motor demand voltage", leftVoltage);
        fieldsTable.recordOutput("Back motor demand voltage", backVoltage);

        io.setRightMotorVoltage(
                MathUtil.clamp(rightVoltage, -OUTTAKE_MOTORS_MAX_VOLTAGE, OUTTAKE_MOTORS_MAX_VOLTAGE));
        io.setLeftMotorVoltage(
                MathUtil.clamp(leftVoltage, -OUTTAKE_MOTORS_MAX_VOLTAGE, OUTTAKE_MOTORS_MAX_VOLTAGE));
        io.setBackMotorVoltage(
            MathUtil.clamp(backVoltage, -BACK_MOTOR_MAX_VOLTAGE, BACK_MOTOR_MAX_VOLTAGE));
    }

    public void stop() {
        setMotorsVoltages(0, 0, 0);
    }
}
