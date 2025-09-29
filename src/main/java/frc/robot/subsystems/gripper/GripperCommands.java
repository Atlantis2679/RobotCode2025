package frc.robot.subsystems.gripper;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class GripperCommands {

  private Gripper gripper;

  public GripperCommands(Gripper gripper){
    this.gripper = gripper;
  }

  public Command outPutL1(){
    return gripper.run(() -> gripper.setMotorsVolt(GripperConstants.L1RightMotorVolt, GripperConstants.L1LeftMotorVolt, GripperConstants.L1BackMotorVolt));
  }

  public Command outPutL2L3(){
    return gripper.run(() -> gripper.setMotorsVolt(GripperConstants.L2L3RightMotorVolt, GripperConstants.L2L3LeftMotorVolt, GripperConstants.L2L3BackMotorVolt));
  }

  public Command inPut(){
    return gripper.run(() -> gripper.setMotorsVolt(0, 0, 0));
  }

  public Command manualController(DoubleSupplier doubleSupplierR, DoubleSupplier doubleSupplierL, DoubleSupplier doubleSupplierB){
    return gripper.run(() -> gripper.setMotorsVolt(doubleSupplierR.getAsDouble(), doubleSupplierL.getAsDouble(), doubleSupplierB.getAsDouble()));
  }

}
