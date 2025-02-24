package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class GripperCommands {
    Gripper gripper;
    
    public GripperCommands(Gripper gripper) {
        this.gripper = gripper;
    }

    public Command loadCoral(double backVoltageForScoring, double rightVoltageForScoring, double leftVoltageForScoring) {
        return gripper.run(() -> gripper.setMotorsVoltage(rightVoltageForScoring, leftVoltageForScoring, backVoltageForScoring))
               .finallyDo(gripper::stop).withName("loadCoral");
    }

    public Command score(double backVoltageForScoring, double rightVoltageForScoring, double leftVoltageForScoring) {
        return gripper.run(() -> gripper.setMotorsVoltage(rightVoltageForScoring, leftVoltageForScoring, backVoltageForScoring))
.withName("scoreL1");
    }

    public Command manualController(DoubleSupplier backSpeed, DoubleSupplier rightSpeed, DoubleSupplier leftSpeed) {
        return gripper.run(() -> 
            gripper.setMotorsVoltage(backSpeed.getAsDouble() * OUTTAKE_MOTORS_MAX_VOLTAGE, 
            rightSpeed.getAsDouble() * OUTTAKE_MOTORS_MAX_VOLTAGE, leftSpeed.getAsDouble() * OUTTAKE_MOTORS_MAX_VOLTAGE))
            .finallyDo(gripper::stop).withName("manualController");
    }

}
