package frc.robot.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.lib.logfields.LogFieldsTable;

public class BuiltInAccelerometerLogged extends BuiltInAccelerometer{
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier z;

    public BuiltInAccelerometerLogged(LogFieldsTable fieldsTable, Range range) {
        super(range);

        x = fieldsTable.addDouble("x", super::getX);
        y = fieldsTable.addDouble("y", super::getY);
        z = fieldsTable.addDouble("z", super::getZ);
    }

    public BuiltInAccelerometerLogged(LogFieldsTable fieldsTable) {
        this(fieldsTable, Range.k8G);
    }

    @Override
    public double getX() {
        return x.getAsDouble();
    }

    @Override
    public double getY() {
        return y.getAsDouble();
    }

    @Override
    public double getZ() {
        return z.getAsDouble();
    }
}
