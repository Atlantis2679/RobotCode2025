package frc.robot.subsystems.gripper.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class GripperIOSim extends GripperIO {
  public GripperIOSim(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  @Override
  protected boolean isCoralIn() {
    return false;
  }

  @Override
  public void setMotorsVolt(double voltRight, double voltLeft, double voltBack) {
  }
  
}
