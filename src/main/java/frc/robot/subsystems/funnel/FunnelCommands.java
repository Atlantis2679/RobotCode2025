package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelCommands {
    public Command setFunnelMotor(double v) {
        return funnel.run(()-> funnel.setVoltage(v));
    }
    public Command stop(){
        return funnel.run(()-> funnel.setVoltage(0));
    }
    private Funnel funnel;
    public FunnelCommands(Funnel funnel){
        this.funnel = funnel;
    }
}
