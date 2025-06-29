package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.swerve.SwerveContants.MAX_VOLTAGE;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import atlantis2679.lib.valueholders.ValueHolder;

public class PivotCommands {
    private final Pivot pivot;

    public PivotCommands(Pivot pivot) {
        this.pivot = pivot;
    }

    public Command moveToAngle(DoubleSupplier desiredAngleDeg) {
        ValueHolder<TrapezoidProfile.State> referenceState = new ValueHolder<TrapezoidProfile.State>(null);
        return pivot.runOnce(() -> {
            pivot.resetPID();
            referenceState.set(new TrapezoidProfile.State(pivot.getAngleDegrees(), pivot.getVelocity()));
        }).andThen(pivot.run(() -> {
            // if (referenceState.get().position == desiredAngleDeg.getAsDouble()
            //         && Math.abs(pivot.getAngleDegrees() - desiredAngleDeg.getAsDouble()) > 40)
            //     referenceState.set(new TrapezoidProfile.State(pivot.getAngleDegrees(), pivot.getVelocity()));

            referenceState.set(pivot.calculateTrapezoidProfile(
                    0.02,
                    referenceState.get(),
                    new TrapezoidProfile.State(desiredAngleDeg.getAsDouble(), 0)));

            double voltage = pivot.calculateFeedForward(
                    referenceState.get().position,
                    referenceState.get().velocity,
                    true);

            pivot.setPivotVoltage(voltage);
        })).withName("pivotMoveToAngle");
    }

    public Command moveToAngle(double angle) {
        return moveToAngle(() -> angle);
    }

    public Command manualController(DoubleSupplier pivotSpeed) {
        return pivot.run(() -> {
            double demandSpeed = pivotSpeed.getAsDouble();
            double feedForward = pivot.calculateFeedForward(pivot.getAngleDegrees(), 0, false);

            pivot.setPivotVoltage(feedForward + demandSpeed * MAX_VOLTAGE);

        }).withName("pivotManualController");
    }
}