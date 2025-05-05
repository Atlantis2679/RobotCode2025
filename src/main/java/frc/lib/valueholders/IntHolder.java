package frc.lib.valueholders;

public class IntHolder {
    private int value;

    public IntHolder(int value) {
        this.value = value;
    }

    public int get() {
        return value;
    }

    public void add(int value) {
        this.value += value;
    }

    public void set(int value) {
        this.value = value;
    }
}
