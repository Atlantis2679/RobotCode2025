package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.swerve.SwerveContants.MAX_VOLTAGE;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.valueholders.ValueHolder;

public class HoodCommands {
        private final Hood hood;

        public HoodCommands(Hood hood) {
        this.hood = hood;
    }

        public Command moveToAngle(DoubleSupplier desiredAngleDeg) {
        ValueHolder<TrapezoidProfile.State> referenceState = new ValueHolder<TrapezoidProfile.State>(null);
        return hood.runOnce(() -> {
            hood.resetPID();
            referenceState.set(new TrapezoidProfile.State(hood.getAngleDegrees(), hood.getVelocity()));
        }).andThen(hood.run(() -> {
            // if (referenceState.get().position == desiredAngleDeg.getAsDouble()
            //         && Math.abs(pivot.getAngleDegrees() - desiredAngleDeg.getAsDouble()) > 40)
            //     referenceState.set(new TrapezoidProfile.State(pivot.getAngleDegrees(), pivot.getVelocity()));

            referenceState.set(hood.calculateTrapezoidProfile(
                    0.02,
                    referenceState.get(),
                    new TrapezoidProfile.State(desiredAngleDeg.getAsDouble(), 0)));

            double voltage = hood.calculateFeedForward(
                    referenceState.get().position,
                    referenceState.get().velocity,
                    true);

            hood.setHoodVoltage(voltage);
        })).withName("pivotMoveToAngle");
    }

    public Command moveToAngle(double angle) {
        return moveToAngle(() -> angle);
    }

    public Command manualController(DoubleSupplier hoodSpeed) {
        return hood.run(() -> {
            double demandSpeed = hoodSpeed.getAsDouble();
            // Feed forward to "ignore" gravity
            double feedForward = hood.calculateFeedForward(hood.getAngleDegrees(), 0, false);

            hood.setHoodVoltage(feedForward + demandSpeed * MAX_VOLTAGE);

        }).withName("pivotManualController");
    }
}
