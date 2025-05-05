package frc.lib.valueholders;

public class LongHolder {
    private long value;

    public LongHolder(long value) {
        this.value = value;
    }

    public long get() {
        return value;
    }

    public void add(long value) {
        this.value += value;
    }

    public void set(long value) {
        this.value = value;
    }
}
