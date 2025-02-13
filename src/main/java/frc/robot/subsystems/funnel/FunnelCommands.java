package frc.robot.subsystems.funnel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelCommands {
    private final Funnel funnel;

    public FunnelCommands(Funnel funnel) {
        this.funnel = funnel;
    }

    public Command loadCoral(double percentageSpeed) {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(percentageSpeed))
            .until(funnel::getIsCoralIn).withName("loadCoral");
    }

    public Command passCoral(double percentageSpeedForLoading, double percentageSpeedForPassing) {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(percentageSpeedForLoading))
                .until(funnel::getIsCoralIn).andThen(funnel.run(() -> funnel.setMotorPercentageSpeed(percentageSpeedForPassing)))
                .until(() -> !funnel.getIsCoralIn()).finallyDo(funnel::stop).withName("passCoral");
    }

    public Command manualController(DoubleSupplier funnelPercentageSpeed) {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(funnelPercentageSpeed.getAsDouble()))
            .finallyDo(funnel::stop).withName("manualController");
    }
}
