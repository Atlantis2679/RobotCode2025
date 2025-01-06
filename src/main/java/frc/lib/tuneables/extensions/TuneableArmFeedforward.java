package frc.lib.tuneables.extensions;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;

public class TuneableArmFeedforward implements Tuneable {
    private ArmFeedforward armFeedforward;

    public TuneableArmFeedforward(double ks, double kg, double kv) {
        armFeedforward = new ArmFeedforward(ks, kg, kv);
    }

    public TuneableArmFeedforward(double ks, double kg, double kv, double ka) {
        armFeedforward = new ArmFeedforward(ks, kg, kv, ka);
    }

    public double calculate(
            double positionRadians,
            double velocityRadPerSec,
            double accelRadPerSecSquared) {
        return armFeedforward.calculate(positionRadians, velocityRadPerSec);
    }

    public double calculate(double positionRadians, double velocity) {
        return armFeedforward.calculate(positionRadians, velocity);
    }

    public double maxAchievableVelocity(double maxVoltage, double angle, double acceleration) {
        return armFeedforward.maxAchievableAcceleration(maxVoltage, angle, acceleration);
    }

    public double minAchievableVelocity(double maxVoltage, double angle, double acceleration) {
        return armFeedforward.minAchievableAcceleration(maxVoltage, angle, acceleration);
    }

    public double maxAchievableAcceleration(double maxVoltage, double angle, double velocity) {
        return armFeedforward.maxAchievableAcceleration(maxVoltage, angle, velocity);
    }

    public double minAchievableAcceleration(double maxVoltage, double angle, double velocity) {
        return armFeedforward.minAchievableAcceleration(maxVoltage, angle, velocity);
    }

    public void setKS(double value) {
        armFeedforward = new ArmFeedforward(value, armFeedforward.getKg(), armFeedforward.getKv(), armFeedforward.getKa());
    }

    public void setKG(double value) {
        armFeedforward = new ArmFeedforward(armFeedforward.getKs(), value, armFeedforward.getKv(), armFeedforward.getKa());
    }

    public void setKV(double value) {
        armFeedforward = new ArmFeedforward(armFeedforward.getKs(), armFeedforward.getKg(), value, armFeedforward.getKa());
    }

    public void setKA(double value) {
        armFeedforward = new ArmFeedforward(armFeedforward.getKs(), armFeedforward.getKg(), armFeedforward.getKv(), value);
    }

    public ArmFeedforward getArmFeedforward() {
        return armFeedforward;
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.setSendableType(SendableType.LIST);
        builder.addDoubleProperty("kS", armFeedforward::getKs, this::setKS);
        builder.addDoubleProperty("kG", armFeedforward::getKg, this::setKG);
        builder.addDoubleProperty("kV", armFeedforward::getKv, this::setKV);
        builder.addDoubleProperty("kA", armFeedforward::getKa, this::setKA);
    }

}
