package frc.robot.subsystems.elevator.io;

import static frc.robot.RobotMap.ELEVATOR_ENCODER_ID;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap.CANBUS;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class ElevatorIOSparkMax extends ElevatorIO{
  private SparkMax rightElevatorMotor = new SparkMax(CANBUS.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private SparkMax leftElevatorMotor = new SparkMax(CANBUS.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(ELEVATOR_ENCODER_ID);

  public ElevatorIOSparkMax(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  //Outputs:

  @Override
  public double getHeightMeters() {
    return encoder.get();
  }
  @Override
  public double getRightMotorCurrent() {
    return rightElevatorMotor.getOutputCurrent();
  }
  @Override
  public double getLeftMotorCurrent() {
    return leftElevatorMotor.getOutputCurrent();
  }
  @Override
  public boolean getIsEncoderConnected() {
    return encoder.isConnected();
  }

  //Inputs:

  @Override
  public void setVoltage(double voltage) {
    rightElevatorMotor.setVoltage(voltage);
    leftElevatorMotor.setVoltage(voltage);
  }
}
