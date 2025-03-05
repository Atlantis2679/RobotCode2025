package frc.lib.tuneables.extensions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;

public class TuneableSimpleMotorFeedforward extends SimpleMotorFeedforward implements Tuneable {
    public TuneableSimpleMotorFeedforward(double ks, double kv, double ka, double dtSeconds) {
        super(ks, kv, ka, dtSeconds);
    }

    public TuneableSimpleMotorFeedforward(double ks, double kv, double ka) {
        super(ks, kv, ka);
    }

    public TuneableSimpleMotorFeedforward(double ks, double kv) {
        super(ks, kv);
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.setSendableType(SendableType.LIST);
        builder.addDoubleProperty("kS", this::getKs, this::setKs);
        builder.addDoubleProperty("kV", this::getKv, this::setKv);
        builder.addDoubleProperty("kA", this::getKa, this::setKa);
    }
}
