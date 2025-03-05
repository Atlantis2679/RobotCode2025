package frc.lib.tuneables.extensions;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;

public class TuneableArmFeedforward extends ArmFeedforward implements Tuneable {
    public TuneableArmFeedforward(double ks, double kg, double kv, double ka, double dtSeconds) {
        super(ks, kg, kv, ka, dtSeconds);
    }

    public TuneableArmFeedforward(double ks, double kg, double kv, double ka) {
        super(ks, kg, kv, ka);
    }

    public TuneableArmFeedforward(double ks, double kg, double kv) {
        super(ks, kg, kv);
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.setSendableType(SendableType.LIST);
        builder.addDoubleProperty("kS", this::getKs, this::setKs);
        builder.addDoubleProperty("kG", this::getKg, this::setKg);
        builder.addDoubleProperty("kV", this::getKv, this::setKv);
        builder.addDoubleProperty("kA", this::getKa, this::setKa);
    }

}
