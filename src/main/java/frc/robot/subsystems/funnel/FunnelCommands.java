package frc.robot.subsystems.funnel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.funnel.FunnelConstants.*;

public class FunnelCommands {
    private final Funnel funnel;

    public FunnelCommands(Funnel funnel) {
        this.funnel = funnel;
    }

    public Command loadCoral() {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(MOTOR_PERCENTAGE_SPEED_FOR_PASSING_CORAL))
            .until(() -> funnel.getIsCoralIn()).finallyDo(() -> funnel.stop()).withName("loadCoral");
    }

    public Command passCoral() {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(MOTOR_PERCENTAGE_SPEED_FOR_PASSING_CORAL))
            .withTimeout(Seconds.of(TIMEOUT_FOR_PASSING_CORAL_SECONDS)).finallyDo(() -> funnel.stop()).withName("passCoral");
    }

    public Command manualController(DoubleSupplier funnelPercentageSpeed) {
        return funnel.run(() -> funnel.setMotorPercentageSpeed(funnelPercentageSpeed.getAsDouble()))
            .finallyDo(() -> funnel.stop()).withName("manualController");
    }
}
