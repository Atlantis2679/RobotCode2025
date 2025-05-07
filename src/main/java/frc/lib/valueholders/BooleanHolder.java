package frc.lib.valueholders;

public class BooleanHolder {
    private boolean value;

    public BooleanHolder(boolean value) {
        this.value = value;
    }

    public boolean get() {
        return value;
    }

    public void not() {
        value = !value;
    }

    public void set(boolean value) {
        this.value = value;
    }
}
