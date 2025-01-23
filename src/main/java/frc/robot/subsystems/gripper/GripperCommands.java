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
        return gripper.run(() -> gripper.setOuttakeMotorsVoltage(
            OUTTAKE_MOTORS_VOLTAGE_FOR_LOADING, OUTTAKE_MOTORS_VOLTAGE_FOR_LOADING))
            .alongWith(gripper.run(() -> gripper.setBackMotorVoltage(BACK_MOTOR_VOLTAGE_FOR_LOADING)))
            .until(gripper::getIsCoralIn).andThen(gripper::stop).withName("loadCoral");
    }

    public Command scoreL1() {
        return gripper.run(() -> gripper.setOuttakeMotorsVoltage(
            OUTTAKE_MOTORS_VOLTAGE_FOR_LOADING, OUTTAKE_MOTORS_VOLTAGE_FOR_LOADING))
            .alongWith(gripper.run(() -> gripper.setBackMotorVoltage(BACK_MOTOR_VOLTAGE_FOR_LOADING)))
            .until(gripper::getIsCoralIn)
            .andThen(manualController(
                () -> RIGHT_OUTTAKE_MOTOR_VOLTAGE_FOR_L1, () -> LEFT_OUTTAKE_MOTOR_VOLTAGE_FOR_L1,
                () -> BACK_MOTOR_VOLTAGE_FOR_L3))
            .until(() -> !gripper.getIsCoralIn()).finallyDo(gripper::stop).withName("scoreL1");
    }

    public Command scoreL3() {
        return gripper.run(() -> gripper.setOuttakeMotorsVoltage(
            OUTTAKE_MOTORS_VOLTAGE_FOR_LOADING, OUTTAKE_MOTORS_VOLTAGE_FOR_LOADING))
            .alongWith(gripper.run(() -> gripper.setBackMotorVoltage(BACK_MOTOR_VOLTAGE_FOR_LOADING)))
            .until(gripper::getIsCoralIn)
            .andThen(manualController(
                () -> OUTTAKE_MOTORS_VOLTAGE_FOR_L3, () -> OUTTAKE_MOTORS_VOLTAGE_FOR_L3,
                () -> BACK_MOTOR_VOLTAGE_FOR_L3))
            .until(() -> !gripper.getIsCoralIn()).finallyDo(gripper::stop).withName("scoreL3");
    }

    public Command manualController(DoubleSupplier rightOutTakeMotorVoltage, DoubleSupplier leftOutTakeMotorVoltage, DoubleSupplier backMotorVoltage) {
        return gripper.run(() -> 
            gripper.setOuttakeMotorsVoltage(rightOutTakeMotorVoltage.getAsDouble(), leftOutTakeMotorVoltage.getAsDouble()))
            .alongWith(gripper.run(() -> gripper.setBackMotorVoltage(backMotorVoltage.getAsDouble())))
            .finallyDo(gripper::stop).withName("manualController");
    }
}
