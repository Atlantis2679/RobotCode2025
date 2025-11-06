package frc.robot.subsystems.funnel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelCommands {
    public Command setFunnelMotor(double v) {
        return funnel.run(()-> funnel.setVoltage(v));
    }

    private Funnel funnel;
    public FunnelCommands(Funnel funnel){
        this.funnel = funnel;
    }
    public Command manualController(DoubleSupplier speed) {
        double demandSpeed = speed.getAsDouble();
        return funnel.run(() -> funnel.setVoltage(demandSpeed*FunnelConstants.PRECENT_TO_VOLTAGE));
    }
}
