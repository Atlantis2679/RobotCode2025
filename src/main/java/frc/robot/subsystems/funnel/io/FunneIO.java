package frc.robot.subsystems.funnel.io;

import java.util.function.BooleanSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class FunneIO extends IOBase{

  public BooleanSupplier isCoralDetected = fields.addBoolean("funnleCoralDetector", this::isCoralDetected);

  protected FunneIO(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }
  
  protected abstract boolean isCoralDetected();

  public abstract void setMotorVolt(double funnleMotor);

}
