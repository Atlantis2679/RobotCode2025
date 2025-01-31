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
            LODING_MOTORS_VOLTAGE, LODING_MOTORS_VOLTAGE))
            .until(gripper::getIsCoralIn).finallyDo(gripper::stop).withName("loadCoral");
    }

    public Command scoreL1() {
        return gripper.run(() -> gripper.setMotorsVoltage(
            LODING_MOTORS_VOLTAGE, LODING_MOTORS_VOLTAGE))
            .until(gripper::getIsCoralIn)
            .andThen(gripper.run(() -> gripper.setMotorsVoltage(
                L1_RIGHT_MOTOR_VOLTAGE, L1_LEFT_MOTOR_VOLTAGE)))
            .until(() -> !gripper.getIsCoralIn()).finallyDo(gripper::stop).withName("scoreL1");
    }

    public Command scoreL3() {
        return gripper.run(() -> gripper.setMotorsVoltage(
            LODING_MOTORS_VOLTAGE, LODING_MOTORS_VOLTAGE))
            .until(gripper::getIsCoralIn)
            .andThen(gripper.run(() -> gripper.setMotorsVoltage(
                L3_MOTORS_VOLTAGE, L3_MOTORS_VOLTAGE)))
            .until(() -> !gripper.getIsCoralIn()).finallyDo(gripper::stop).withName("scoreL3");
    }

    public Command manualController(DoubleSupplier rightMotorVoltage, DoubleSupplier leftMotorVoltage) {
        return gripper.run(() -> 
            gripper.setMotorsVoltage(rightMotorVoltage.getAsDouble(), leftMotorVoltage.getAsDouble()))
            .finallyDo(gripper::stop).withName("manualController");
    }
}
