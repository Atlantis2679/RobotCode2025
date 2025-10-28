package frc.robot.subsystems.elevator.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class ElevatorIOSim extends ElevatorIO{
  public ElevatorIOSim(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  //Outputs:

  @Override
  public double getEncoderAngle() {
    return 0;
  }
  @Override
  public double getRightMotorCurrent(){
    return 0;
  }
  @Override
  public double getLeftMotorCurrent() {
    return 0;
  }

  //Inputs:

  @Override
  public void setVoltage(double voltage) {}
}
