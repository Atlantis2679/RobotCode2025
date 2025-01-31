package frc.robot.subsystems.funnel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.subsystems.funnel.FunnelConstants.*;

public class FunnelCommands {
    private final Funnel funnel;

    public FunnelCommands(Funnel funnel) {
        this.funnel = funnel;
    }

    public Command loadCoral() {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(MOTORS_PERCENTAGE_SPEED_LOADING))
            .until(funnel::getIsCoralIn).finallyDo(funnel::stop).withName("loadCoral");
    }

    public Command passCoral() {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(MOTORS_PERCENTAGE_SPEED_LOADING))
                .until(funnel::getIsCoralIn).andThen(funnel.run(() -> funnel.setMotorPercentageSpeed(MOTORS_PERCENTAGE_SPEED_PASSING))
                .until(() -> !funnel.getIsCoralIn())).finallyDo(funnel::stop).withName("passCoral");
    }

    public Command manualController(DoubleSupplier funnelPercentageSpeed) {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(funnelPercentageSpeed.getAsDouble()))
            .finallyDo(funnel::stop).withName("manualController");
    }
}
