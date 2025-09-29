package frc.robot.subsystems.gripper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.gripper.io.GripeprIOSparkMax;
import frc.robot.subsystems.gripper.io.GripperIO;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Gripper extends SubsystemBase{

  GripperIO gripeprIO;
  Debouncer debouncer;
  LogFieldsTable fieldsTable;

  public Gripper(){
    debouncer = new Debouncer(GripperConstants.debounceTimeSec);
    fieldsTable = new LogFieldsTable(getName());
    if(Robot.isReal()){
      gripeprIO  = new GripeprIOSparkMax(fieldsTable);
    }
    
  }

  public void stop(){
    gripeprIO.setMotorsVolt(0,0,0);
  }

  public void setMotorsVolt(double voltR, double voltL, double voltB){
    gripeprIO.setMotorsVolt(voltR,voltL,voltB);
  }

  public boolean getisCoralInPostDebouncer(){
    return debouncer.calculate(gripeprIO.isCoralIn.getAsBoolean());
  }


  
}
