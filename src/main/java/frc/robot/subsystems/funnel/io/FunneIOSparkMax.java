package frc.robot.subsystems.funnel.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class FunneIOSparkMax extends FunneIO{

  private DigitalInput beamBreak = new DigitalInput(9);

  private SparkMax funnleMotor = new SparkMax(10, MotorType.kBrushless);

  public FunneIOSparkMax(LogFieldsTable logFieldsTable){
    super(logFieldsTable);
  }
  
  public boolean isCoralDetected(){
    return beamBreak.get();
  }

  public void setMotorVolt(double motorvolt){
    funnleMotor.setVoltage(motorvolt);
  }

}