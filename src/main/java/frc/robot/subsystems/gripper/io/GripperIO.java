package frc.robot.subsystems.gripper.io;

import java.util.function.BooleanSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class GripperIO extends IOBase {

  public BooleanSupplier isCoralIn = fields.addBoolean("iscoralIn" ,this::isCoralIn);

  public GripperIO(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  protected abstract boolean isCoralIn();

  public abstract void setMotorsVolt(double voltRight, double voltLeft, double voltBack);

}
