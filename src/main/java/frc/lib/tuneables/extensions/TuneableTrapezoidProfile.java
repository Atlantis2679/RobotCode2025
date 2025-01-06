package frc.lib.tuneables.extensions;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;

public class TuneableTrapezoidProfile implements Tuneable {
    private TrapezoidProfile trapezoidProfile;
    private Constraints constraints;

    public TuneableTrapezoidProfile(Constraints constraints) {
        this.constraints = constraints;
        trapezoidProfile = new TrapezoidProfile(constraints);
    }

    public State calculate(double t, State current, State goal) {
        return trapezoidProfile.calculate(t, current, goal);
    }

    public double timeLeftUntil(double target) {
        return trapezoidProfile.timeLeftUntil(target);
    }

    public double totalTime() {
        return trapezoidProfile.totalTime();
    }

    public boolean isFinished(double t) {
        return trapezoidProfile.isFinished(t);
    }

    public double getMaxVelocity() {
        return constraints.maxVelocity;
    }

    public double getMaxAcceleration() {
        return constraints.maxAcceleration;
    }

    public void setMaxVelocity(double value) {
        constraints = new Constraints(value, constraints.maxAcceleration);
        trapezoidProfile = new TrapezoidProfile(constraints);
    }

    public void setMaxAcceleration(double value) {
        constraints = new Constraints(constraints.maxVelocity, value);
        trapezoidProfile = new TrapezoidProfile(constraints);
    }

    public TrapezoidProfile getTrapezoidProfile() {
        return trapezoidProfile;
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.addDoubleProperty("Max Velocity", this::getMaxVelocity, this::setMaxVelocity);
        builder.addDoubleProperty("Max Acceleration", this::getMaxAcceleration, this::setMaxAcceleration);
    }
}
