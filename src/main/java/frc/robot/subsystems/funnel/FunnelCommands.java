package frc.robot.subsystems.funnel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelCommands {
    private Funnel funnel;

    public FunnelCommands(Funnel funnel){
        this.funnel = funnel;
    }

    public Command passCorral(double speed) {
        return funnel.run(() -> funnel.setMotorSpeed(speed))
            .finallyDo(funnel::stop).withName("Pass Corral");
    }

    public Command manualController(DoubleSupplier speed){
        return funnel.run(() -> funnel.setMotorSpeed(speed.getAsDouble()))
            .finallyDo(funnel::stop).withName("Manual Controller");
    }
}
