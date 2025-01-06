package frc.lib.tuneables.sendableproperties;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.logfields.LogFieldsTable;

public class BooleanTuneableProperty extends TuneableProperty {
    private final Supplier<boolean[]> field;
    private boolean[] valueFromNetwork = {};
    private final BooleanConsumer setter;

    public BooleanTuneableProperty(
            String key,
            BooleanSupplier getter,
            BooleanConsumer setter,
            LogFieldsTable fieldsTable,
            SendableBuilder sendableBuilder) {
        this.setter = setter;

        field = fieldsTable.addBooleanArray(key, () -> {
            boolean[] newValue = valueFromNetwork;
            valueFromNetwork = new boolean[] {};
            return newValue;
        });
        fieldsTable.update();

        sendableBuilder.addBooleanProperty(key, getter, value -> valueFromNetwork = new boolean[] { value });
    }

    @Override
    protected void updateSetter() {
        if (field.get().length != 0) {
            setter.accept(field.get()[0]);
        }
    }
}
