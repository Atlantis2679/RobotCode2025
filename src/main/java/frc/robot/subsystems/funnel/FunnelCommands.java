package frc.robot.subsystems.funnel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
    
public class FunnelCommands {
    private final Funnel funnel;

    public FunnelCommands(Funnel funnel) {
        this.funnel = funnel;
    }

    public Command loadCoral(double percentageSpeedLoading) {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(percentageSpeedLoading))
            .until(funnel::getIsCoralIn).finallyDo(funnel::stop).withName("loadCoral");
    }

    public Command passCoral(double percentageSpeedLoading, double percentageSpeedPassing) {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(percentageSpeedLoading))
                .until(funnel::getIsCoralIn).andThen(funnel.run(() -> funnel.setMotorPercentageSpeed(percentageSpeedPassing)))
                .finallyDo(funnel::stop).withName("passCoral");
    }

    public Command manualController(DoubleSupplier funnelPercentageSpeed) {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(funnelPercentageSpeed.getAsDouble()))
            .finallyDo(funnel::stop).withName("manualController");
    }
}
