package frc.robot.subsystems.funnel.funnelIO;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class FunnelIOSim extends FunnelIO {

    public FunnelIOSim(LogFieldsTable logFieldsTable){
        super(logFieldsTable);
    }

    @Override
    public boolean getBeamBreak(){
        return false;
    }




}
