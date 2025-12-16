package frc.robot.subsystems.funnel.funnelIO;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class FunnelIOSim extends FunnelIO {

    public FunnelIOSim(LogFieldsTable logFieldsTable){
        super(logFieldsTable);
    }

    //Output methods:
    @Override
    public boolean getBeamBreak(){
        return false;
    }

    @Override
    public double getMotorCurrent(){
        return 0;
    }
    
    //Input methods:
    public void setPrecentageSpeed(double speed){} 



}
