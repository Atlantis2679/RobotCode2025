package frc.robot.subsystems.funnel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.funnel.io.FunneIO;
import frc.robot.subsystems.funnel.io.FunneIOSparkMax;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Funnel extends SubsystemBase{

  LogFieldsTable fieldsTable;
  FunneIO funnleIO;
  Debouncer debouncer;

  private double lastDesiredVoltage = 0;

  public Funnel(){
    debouncer = new Debouncer(FunnelConstants.debounceTimeSec);
    fieldsTable = new LogFieldsTable(getName());
    if(Robot.isReal()){
      funnleIO = new FunneIOSparkMax(fieldsTable);
    }
  }

  @Override
  public void periodic() {
    fieldsTable.recordOutput("desired voltage", lastDesiredVoltage);
  }

  public void stop(){
    funnleIO.setMotorVolt(0);
  }

  public void setMotorVolt(double volt){
    lastDesiredVoltage = volt;
    funnleIO.setMotorVolt(volt);
  }

  public boolean getIsCoralDetectedPostDebouncer(){
    return debouncer.calculate(funnleIO.isCoralDetected.getAsBoolean());
  }
  
}
