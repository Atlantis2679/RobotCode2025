package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class GripperCommands {
    Gripper gripper;

    public GripperCommands(Gripper gripper) {
        this.gripper = gripper;
    }

    public Command spin(double backVoltageForScoring, double rightVoltageForScoring, double leftVoltageForScoring) {
        return gripper.run(
                () -> gripper.setMotorsVoltages(rightVoltageForScoring, leftVoltageForScoring, backVoltageForScoring))
                .finallyDo(gripper::stop).withName("spin gripper");
    }

    public Command manualController(DoubleSupplier backSpeed, DoubleSupplier rightSpeed, DoubleSupplier leftSpeed) {
        return gripper.run(() -> gripper.setMotorsVoltages(
                backSpeed.getAsDouble() * BACK_MOTOR_MAX_VOLTAGE,
                rightSpeed.getAsDouble() * OUTTAKE_MOTORS_MAX_VOLTAGE,
                leftSpeed.getAsDouble() * OUTTAKE_MOTORS_MAX_VOLTAGE))
                .finallyDo(gripper::stop).withName("gripper manual controller");
    }

}
