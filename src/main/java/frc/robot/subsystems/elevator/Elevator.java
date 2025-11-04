package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.io.ElevatorIO;
import frc.robot.subsystems.elevator.io.ElevatorIOSim;
import frc.robot.subsystems.elevator.io.ElevatorIOSparkMax;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Elevator extends SubsystemBase {
  private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

  private final ElevatorIO io = Robot.isReal() ? new ElevatorIOSparkMax(fieldsTable) : new ElevatorIOSim(fieldsTable);

  public Elevator() {}

  @Override
  public void periodic() {
    fieldsTable.recordOutput("Encoder angle", io.getEncoderAngle());
    fieldsTable.recordOutput("Right motor current", io.getRightMotorCurrent());
    fieldsTable.recordOutput("Left motor current", io.getLeftMotorCurrent());
  }
  public void setVoltage(double voltage){
    fieldsTable.recordOutput("Motor demand voltage", voltage);
    io.setVoltage(voltage);
  }
  public double getEncoderAngle(){
      return io.getEncoderAngle();
  }
  public void stop(){
    io.setVoltage(0);
  }
}
