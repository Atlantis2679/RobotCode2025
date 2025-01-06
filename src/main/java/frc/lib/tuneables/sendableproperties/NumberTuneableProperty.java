package frc.lib.tuneables.sendableproperties;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.logfields.LogFieldsTable;

public class NumberTuneableProperty extends TuneableProperty {
    private final Supplier<double[]> field;
    private double[] valueFromNetwork = {};
    private final DoubleConsumer setter;

    public NumberTuneableProperty(
            String key,
            DoubleSupplier getter,
            DoubleConsumer setter,
            LogFieldsTable fieldsTable,
            SendableBuilder sendableBuilder) {
        this.setter = setter;

        field = fieldsTable.addDoubleArray(key, () -> {
            double[] newValue = valueFromNetwork;
            valueFromNetwork = new double[] {};
            return newValue;
        });
        fieldsTable.update();

        sendableBuilder.addDoubleProperty(key, getter, value -> valueFromNetwork = new double[] { value });
    }

    @Override
    protected void updateSetter() {
        if (field.get().length != 0) {
            setter.accept(field.get()[0]);
        }
    }
}
