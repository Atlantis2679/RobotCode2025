package frc.robot.subsystems.gripper;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class GripperCommands {
    Gripper gripper;
    
    public GripperCommands(Gripper gripper) {
        this.gripper = gripper;
    }

    public Command loadCoral(double voltage) {
        return gripper.run(() -> gripper.setMotorsVoltage(voltage, voltage))
            .until(gripper::getIsCoralIn).finallyDo(gripper::stop).withName("loadCoral");
    }

    public Command scoreL1(double voltageForLoading, double rightVoltageForScoring, double leftVoltageForScoring) {
        return gripper.run(() -> gripper.setMotorsVoltage(voltageForLoading, voltageForLoading))
            .until(gripper::getIsCoralIn)
            .andThen(gripper.run(() -> gripper.setMotorsVoltage(rightVoltageForScoring, leftVoltageForScoring)))
            .until(() -> !gripper.getIsCoralIn())
            .finallyDo(gripper::stop).withName("scoreL1");
    }

    public Command scoreL3(double voltageForLoading, double voltageForScoring) {
        return gripper.run(() -> gripper.setMotorsVoltage(voltageForLoading, voltageForLoading))
            .until(gripper::getIsCoralIn)
            .andThen(gripper.run(() -> gripper.setMotorsVoltage(voltageForScoring, voltageForScoring)))
            .until(() -> !gripper.getIsCoralIn())
            .finallyDo(gripper::stop).withName("scoreL3");
    }

    public Command manualController(DoubleSupplier rightMotorVoltage, DoubleSupplier leftMotorVoltage) {
        return gripper.run(() -> gripper.setMotorsVoltage(rightMotorVoltage.getAsDouble(), leftMotorVoltage.getAsDouble()))
            .finallyDo(gripper::stop).withName("manualController");
    }
}
