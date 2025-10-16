package frc.robot.subsystems.funnel.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class FunnelIOSim extends FunnelIO {

   public FunnelIOSim(LogFieldsTable logFieldsTable){
      super(logFieldsTable);
   }
   protected boolean getBeamBreak() {
      return false;
   }
   public void setVoltage(double voltage){}

    
}
