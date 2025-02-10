package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.swerve.SwerveContants.MAX_VOLTAGE;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnel.Funnel;

public class GripperCommands {
    Gripper gripper;
    Funnel funnel;
    
    public GripperCommands(Gripper gripper, Funnel funnel) {
        this.gripper = gripper;
        this.funnel = funnel;
    }

    public Command loadCoral(double voltage) {
        return gripper.run(() -> gripper.setMotorsVoltage(voltage, voltage))
               .until(gripper::getIsCoralIn).finallyDo(gripper::stop).withName("loadCoral");
    }

    public Command score(double voltageForLoading, double rightVoltageForScoring, double leftVoltageForScoring) {
        return loadCoral(voltageForLoading)
            .until(gripper::getIsCoralIn)
            .andThen(gripper.run(() -> gripper.setMotorsVoltage(rightVoltageForScoring, leftVoltageForScoring)))
            .until(() -> !gripper.getIsCoralIn())
            .finallyDo(gripper::stop).withName("scoreL1");
    }

    public Command manualController(DoubleSupplier rightSpeed, DoubleSupplier leftSpeed) {
        return gripper.run(() -> 
            gripper.setMotorsVoltage(rightSpeed.getAsDouble() * MAX_VOLTAGE, leftSpeed.getAsDouble() * MAX_VOLTAGE))
            .finallyDo(gripper::stop).withName("manualController");
    }
}
