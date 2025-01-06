package frc.lib.tuneables.sendableproperties;

import java.util.ArrayList;
import java.util.List;

public abstract class TuneableProperty {
    private static List<TuneableProperty> createdProperties = new ArrayList<>();
    
    public TuneableProperty() {
        createdProperties.add(this);
    }

    public static void updateAll() {
        for (TuneableProperty property : createdProperties) {
            property.updateSetter();
        }
    }

    protected abstract void updateSetter();
}