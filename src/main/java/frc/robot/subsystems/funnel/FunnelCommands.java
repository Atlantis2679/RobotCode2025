package frc.robot.subsystems.funnel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
    
public class FunnelCommands {
    private final Funnel funnel;

    public FunnelCommands(Funnel funnel) {
        this.funnel = funnel;
    }

    public Command passCoral(double precentageSpeed) {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(precentageSpeed))
                .finallyDo(funnel::stop).withName("passCoral");
    }

    public Command manualController(DoubleSupplier funnelPercentageSpeed) {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(funnelPercentageSpeed.getAsDouble()))
            .finallyDo(funnel::stop).withName("manualController");
    }
}
