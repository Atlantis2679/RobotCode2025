package frc.lib.logfields;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.logfields.logfields.BooleanLogField;
import frc.lib.logfields.logfields.DoubleLogField;
import frc.lib.logfields.logfields.FloatLogField;
import frc.lib.logfields.logfields.IntegerLogField;
import frc.lib.logfields.logfields.LongLogField;
import frc.lib.networkalerts.GenericError;
import frc.lib.logfields.logfields.LogField;

public class LogFieldsTable implements LoggableInputs {
    private final static ArrayList<LogFieldsTable> createdTables = new ArrayList<>();

    private final String name;
    private final String prefix;
    private final List<LoggableInputs> fields = new ArrayList<>();
    private Runnable periodicBeforeFields = null;

    public LogFieldsTable(String name) {
        this.name = name;
        prefix = name + "/";
    }

    public static void updateAllTables() {
        for (LogFieldsTable fieldsTable : createdTables) {
            fieldsTable.update();
        }
    }

    public void update() {
        if (periodicBeforeFields != null && !Logger.hasReplaySource()) {
            periodicBeforeFields.run();
        }

        try{
            Logger.processInputs(name, this);
        } catch(Exception e) {
            DriverStation.reportError("error with proccessing " + name + " table.", e.getStackTrace());
        }
    }

    public LogFieldsTable getSubTable(String name) {
        return new LogFieldsTable(prefix + name);
    }

    @Override
    public void toLog(LogTable table) {
        for (LoggableInputs field : fields) {
            field.toLog(table);
        }
    }

    @Override
    public void fromLog(LogTable table) {
        for (LoggableInputs field : fields) {
            field.fromLog(table);
        }
    }

    public void setPeriodicBeforeFields(Runnable periodicRunnable) {
        this.periodicBeforeFields = periodicRunnable;
    }

    private <T extends LoggableInputs> T registerField(T field) {
        if (fields.isEmpty()) {
            // only add the table when it has fields, to not save each subTable created in
            // periodic for outputs recording (because that leads to saving tons of unneeded
            // fields table that doesn't have any fields).
            createdTables.add(this);
        }
        fields.add(field);
        return field;
    }

    public Supplier<byte[]> addRaw(
            String name,
            Supplier<byte[]> valueSupplier,
            byte[] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<byte[]> addRaw(String name, Supplier<byte[]> valueSupplier) {
        return addRaw(name, valueSupplier, new byte[0]);
    }

    public Supplier<byte[][]> addRawMatrix(
            String name,
            Supplier<byte[][]> valueSupplier,
            byte[][] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<byte[][]> addRawMatrix(String name, Supplier<byte[][]> valueSupplier) {
        return addRawMatrix(name, valueSupplier, new byte[0][0]);
    }

    public BooleanSupplier addBoolean(
            String name,
            BooleanSupplier valueSupplier,
            boolean defaultValue) {
        return registerField(new BooleanLogField(name, valueSupplier, defaultValue));
    }

    public BooleanSupplier addBoolean(String name, BooleanSupplier valueSupplier) {
        return addBoolean(name, valueSupplier, false);
    }

    public Supplier<Integer> addInteger(
        String name,
        Supplier<Integer> valueSupplier,
        int defaultValue) {
        return registerField(new IntegerLogField(name, valueSupplier, defaultValue));
    }

    public Supplier<Integer> addInteger(String name, Supplier<Integer> valueSupplier) {
        return addInteger(name, valueSupplier, 0);
    }

    public LongSupplier addLong(
            String name,
            LongSupplier valueSupplier,
            long defaultValue) {
        return registerField(new LongLogField(name, valueSupplier, defaultValue));
    }

    public LongSupplier addLong(String name, LongSupplier valueSupplier) {
        return addLong(name, valueSupplier, 0);
    }

    public FloatSupplier addFloat(
            String name,
            FloatSupplier valueSupplier,
            float defaultValue) {
        return registerField(new FloatLogField(name, valueSupplier, defaultValue));
    }

    public FloatSupplier addFloat(String name, FloatSupplier valueSupplier) {
        return addFloat(name, valueSupplier, 0);
    }

    public DoubleSupplier addDouble(
            String name,
            DoubleSupplier valueSupplier,
            double defaultValue) {
        return registerField(new DoubleLogField(name, valueSupplier, defaultValue));
    }

    public DoubleSupplier addDouble(String name, DoubleSupplier valueSupplier) {
        return addDouble(name, valueSupplier, 0);
    }

    public Supplier<String> addString(
            String name,
            Supplier<String> valueSupplier,
            String defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<String> addString(String name, Supplier<String> valueSupplier) {
        return addString(name, valueSupplier, "");
    }

    public Supplier<boolean[]> addBooleanArray(
            String name,
            Supplier<boolean[]> valueSupplier,
            boolean[] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<boolean[]> addBooleanArray(String name, Supplier<boolean[]> valueSupplier) {
        return addBooleanArray(name, valueSupplier, new boolean[0]);
    }

    public Supplier<int[]> addIntegerArray(
        String name,
        Supplier<int[]> valueSupplier,
        int[] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<int[]> addIntegerArray(String name, Supplier<int[]> valueSupplier) {
        return addIntegerArray(name, valueSupplier, new int[0]);
    }

    public Supplier<long[]> addLongArray(
            String name,
            Supplier<long[]> valueSupplier,
            long[] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<long[]> addLongArray(String name, Supplier<long[]> valueSupplier) {
        return addLongArray(name, valueSupplier, new long[0]);
    }

    public Supplier<float[]> addFloatArray(
            String name,
            Supplier<float[]> valueSupplier,
            float[] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<float[]> addFloatArray(String name, Supplier<float[]> valueSupplier) {
        return addFloatArray(name, valueSupplier, new float[0]);
    }

    public Supplier<double[]> addDoubleArray(
            String name,
            Supplier<double[]> valueSupplier,
            double[] defaultValue) {

        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<double[]> addDoubleArray(String name, Supplier<double[]> valueSupplier) {
        return addDoubleArray(name, valueSupplier, new double[0]);
    }

    public Supplier<String[]> addStringArray(
            String name,
            Supplier<String[]> valueSupplier,
            String[] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<String[]> addStringArray(String name, Supplier<String[]> valueSupplier) {
        return addStringArray(name, valueSupplier, new String[0]);
    }

    public Supplier<boolean[][]> addBooleanMatrix(
            String name,
            Supplier<boolean[][]> valueSupplier,
            boolean[][] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<boolean[][]> addBooleanMatrix(String name, Supplier<boolean[][]> valueSupplier) {
        return addBooleanMatrix(name, valueSupplier, new boolean[0][0]);
    }

    public Supplier<int[][]> addIntegerMatrix(
        String name,
        Supplier<int[][]> valueSupplier,
        int[][] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<int[][]> addIntegerMatrix(String name, Supplier<int[][]> valueSupplier) {
        return addIntegerMatrix(name, valueSupplier, new int[0][0]);
    }

    public Supplier<long[][]> addLongMatrix(
            String name,
            Supplier<long[][]> valueSupplier,
            long[][] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<long[][]> addLongMatrix(String name, Supplier<long[][]> valueSupplier) {
        return addLongMatrix(name, valueSupplier, new long[0][0]);
    }

    public Supplier<float[][]> addFloatMatrix(
            String name,
            Supplier<float[][]> valueSupplier,
            float[][] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<float[][]> addFloatMatrix(String name, Supplier<float[][]> valueSupplier) {
        return addFloatMatrix(name, valueSupplier, new float[0][0]);
    }

    public Supplier<double[][]> addDoubleMatrix(
            String name,
            Supplier<double[][]> valueSupplier,
            double[][] defaultValue) {

        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<double[][]> addDoubleMatrix(String name, Supplier<double[][]> valueSupplier) {
        return addDoubleMatrix(name, valueSupplier, new double[0][0]);
    }

    public Supplier<String[][]> addStringMatrix(
            String name,
            Supplier<String[][]> valueSupplier,
            String[][] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public Supplier<String[][]> addStringMatrix(String name, Supplier<String[][]> valueSupplier) {
        return addStringMatrix(name, valueSupplier, new String[0][0]);
    }

    public Supplier<GenericError> addGenericError(
        String name,
        Supplier<GenericError> valueSupplier,
        GenericError defaultValue) {
            LogFieldsTable subTable = getSubTable(name);
            Supplier<String> message = subTable.addString(name, () -> valueSupplier.get().message(), defaultValue.message());
            Supplier<String> alertGroup = subTable.addString(name, () -> valueSupplier.get().alertGroup(), defaultValue.alertGroup());
            Supplier<String> errorType = subTable.addString(name, () -> valueSupplier.get().errorType(), defaultValue.errorType());
            Supplier<String> details = subTable.addString(name, () -> valueSupplier.get().details(), defaultValue.details());
            Supplier<Integer> errorCode = subTable.addInteger(name, () -> valueSupplier.get().errorCode(), defaultValue.errorCode());
            BooleanSupplier isActive = subTable.addBoolean(name, () -> valueSupplier.get().isActive(), defaultValue.isActive());
            Supplier<String> alertType = subTable.addString(name, () -> valueSupplier.get().alertType().name(), defaultValue.alertType().name());
            return () -> new GenericError(message.get(), alertGroup.get(), errorType.get(), details.get(), errorCode.get(), isActive.getAsBoolean(), AlertType.valueOf(alertType.get()));
    }

    public Supplier<GenericError> addGenericError(String name, Supplier<GenericError> valueSupplier) {
        return addGenericError(name, valueSupplier, new GenericError("none", valueSupplier.get().alertGroup(), "none", "none", 0, false, valueSupplier.get().alertType()));
    }

    public <T extends WPISerializable> Supplier<T> addObject(
            String name,
            Supplier<T> valueSupplier,
            T defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    @SuppressWarnings("unchecked")
    public <T extends StructSerializable> Supplier<T[]> addObjectArray(
            String name,
            Supplier<T[]> valueSupplier,
            T[] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public <T extends StructSerializable> Supplier<T[][]> addObjectMatrix(
            String name,
            Supplier<T[][]> valueSupplier,
            T[][] defaultValue) {
        return registerField(new LogField<>(name, valueSupplier, LogTable::get, LogTable::put, defaultValue));
    }

    public void recordOutput(String name, byte[] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, byte[][] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, boolean value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, int value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, long value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, float value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, double value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, String value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, boolean[] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, int[] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, long[] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, float[] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, double[] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, String[] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, boolean[][] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, int[][] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, long[][] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, float[][] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, double[][] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, String[][] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public <T extends WPISerializable> void recordOutput(String name, T value) {
        Logger.recordOutput(prefix + name, value);
    }

    @SuppressWarnings("unchecked")
    public <T extends StructSerializable> void recordOutput(String name, T... value) {
        Logger.recordOutput(prefix + name, value);
    }

    public <T extends StructSerializable> void recordOutput(String name, T[][] value) {
        Logger.recordOutput(prefix + name, value);
    }

    public <T extends WPISerializable> void recordOutput(String name, LoggedMechanism2d value) {
        Logger.recordOutput(prefix + name, value);
    }
}