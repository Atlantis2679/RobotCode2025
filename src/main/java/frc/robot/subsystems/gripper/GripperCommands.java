package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class GripperCommands {
    Gripper gripper;
    
    public GripperCommands(Gripper gripper) {
        this.gripper = gripper;
    }

    public Command loadCoral() {
        return gripper.run(() -> gripper.setMotorsVoltage(
            LODING_OUTTAKE_MOTORS_VOLTAGE, LODING_OUTTAKE_MOTORS_VOLTAGE, LODING_BACK_MOTOR_VOLTAGE))
            .until(gripper::getIsCoralIn).finallyDo(gripper::stop).withName("loadCoral");
    }

    public Command scoreL1() {
        return gripper.run(() -> gripper.setMotorsVoltage(
            LODING_OUTTAKE_MOTORS_VOLTAGE, LODING_OUTTAKE_MOTORS_VOLTAGE, LODING_BACK_MOTOR_VOLTAGE))
            .until(gripper::getIsCoralIn)
            .andThen(gripper.run(() -> gripper.setMotorsVoltage(
                L1_RIGHT_OUTTAKE_MOTOR_VOLTAGE, L1_LEFT_OUTTAKE_MOTOR_VOLTAGE, L1_BACK_MOTOR_VOLTAGE)))
            .until(() -> !gripper.getIsCoralIn()).finallyDo(gripper::stop).withName("scoreL1");
    }

    public Command scoreL3() {
        return gripper.run(() -> gripper.setMotorsVoltage(
            LODING_OUTTAKE_MOTORS_VOLTAGE, LODING_OUTTAKE_MOTORS_VOLTAGE, LODING_BACK_MOTOR_VOLTAGE))
            .until(gripper::getIsCoralIn)
            .andThen(gripper.run(() -> gripper.setMotorsVoltage(
                L3_OUTTAKE_MOTORS_VOLTAGE, L3_OUTTAKE_MOTORS_VOLTAGE, L3_BACK_MOTOR_VOLTAGE)))
            .until(() -> !gripper.getIsCoralIn()).finallyDo(gripper::stop).withName("scoreL3");
    }

    public Command manualController(DoubleSupplier rightOutTakeMotorVoltage, DoubleSupplier leftOutTakeMotorVoltage, DoubleSupplier backMotorVoltage) {
        return gripper.run(() -> 
            gripper.setMotorsVoltage(rightOutTakeMotorVoltage.getAsDouble(), leftOutTakeMotorVoltage.getAsDouble(), backMotorVoltage.getAsDouble()))
            .finallyDo(gripper::stop).withName("manualController");
    }
}
