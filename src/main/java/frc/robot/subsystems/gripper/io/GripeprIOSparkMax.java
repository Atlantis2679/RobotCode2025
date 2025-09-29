package frc.robot.subsystems.gripper.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class GripeprIOSparkMax extends GripperIO {
  private DigitalInput beamBreak = new DigitalInput(0);

  private SparkMax rightMotor = new SparkMax(14, MotorType.kBrushless);
  private SparkMax leftMotor = new SparkMax(13, MotorType.kBrushless);
  private SparkMax backMotor = new SparkMax(12, MotorType.kBrushless);

  public GripeprIOSparkMax(LogFieldsTable fieldsTable) {
    super(fieldsTable);  
  }

  @Override
  protected boolean isCoralIn() {
    return beamBreak.get();
  }

  @Override
  public void setMotorsVolt(double voltRight, double voltLeft, double voltBack) {
    rightMotor.setVoltage(voltRight);
    leftMotor.setVoltage(voltLeft);
    backMotor.setVoltage(voltBack);
  }
}
