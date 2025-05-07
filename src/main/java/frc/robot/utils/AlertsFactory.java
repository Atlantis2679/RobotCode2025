package frc.robot.utils;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.networkalerts.NetworkPeriodicAlert;

public final class AlertsFactory {
    public static NetworkPeriodicAlert[] revMotor(Supplier<REVLibError> configErrorSupplier,
            Supplier<Warnings> warningSupplier, Supplier<Faults> errorSupplier, String prefix) {
        NetworkPeriodicAlert[] periodicAlerts = new NetworkPeriodicAlert[3];
        periodicAlerts[0] = revError(configErrorSupplier, prefix);
        periodicAlerts[1] = sparkMaxWarning(warningSupplier, prefix);
        periodicAlerts[2] = sparkMaxError(errorSupplier, prefix);
        return periodicAlerts;
    }

    public static NetworkPeriodicAlert revError(Supplier<REVLibError> errorSupplier, String prefix) {
        return new NetworkPeriodicAlert(null, () -> prefix + ": " + errorSupplier.get().name(),
            AlertType.kError, () -> errorSupplier.get().value != 0);
    }

    public static NetworkPeriodicAlert sparkMaxWarning(Supplier<Warnings> warningSupplier, String prefix) {
        Supplier<String> messageSupplier = () -> {
          String message = prefix + ": ";
          Warnings warning = warningSupplier.get();
          if (warning.brownout) message += "brownout, ";
          if (warning.escEeprom) message += "escEeprom, ";
          if (warning.extEeprom) message += "extEeprom, ";
          if (warning.hasReset) message += "hasReset, ";
          if (warning.other) message += "other, ";
          if (warning.overcurrent) message += "overcurrent, ";
          if (warning.sensor) message += "sensor, ";
          if (warning.stall) message += "stall, ";
          return message;
        };
        return new NetworkPeriodicAlert(null, messageSupplier, AlertType.kWarning, () -> warningSupplier.get().rawBits != 0);
    }

    public static NetworkPeriodicAlert sparkMaxError(Supplier<Faults> errorSupplier, String prefix) {
        Supplier<String> messageSupplier = () -> {
          String message = prefix + ": ";
          Faults error = errorSupplier.get();
          if (error.can) message += "can, ";
          if (error.escEeprom) message += "escEeprom, ";
          if (error.firmware) message += "firmware, ";
          if (error.gateDriver) message += "gateDriver, ";
          if (error.motorType) message += "motorType, ";
          if (error.other) message += "other, ";
          if (error.sensor) message += "sensor, ";
          if (error.temperature) message += "temperature, ";
          return message;
        };
        return new NetworkPeriodicAlert(null, messageSupplier, AlertType.kError, () -> errorSupplier.get().rawBits != 0);
    }

    public static NetworkPeriodicAlert phoenixWarning(Supplier<StatusCode> statusSupplier, String prefix) {
        return new NetworkPeriodicAlert(null, () -> prefix + ": " + statusSupplier.get().getName(),
            AlertType.kWarning , () -> statusSupplier.get().isWarning());
    }

    public static NetworkPeriodicAlert phoenixError(Supplier<StatusCode> statusSupplier, String prefix) {
        return new NetworkPeriodicAlert(null, () -> prefix + ": " + statusSupplier.get().getName(),
            AlertType.kError , () -> statusSupplier.get().isError());
    }

    public static NetworkPeriodicAlert[] phoenixMotor(Supplier<StatusCode> statusSupplier, String prefix) {
        NetworkPeriodicAlert[] periodicAlerts = new NetworkPeriodicAlert[2];
        periodicAlerts[0] = phoenixWarning(statusSupplier, prefix);
        periodicAlerts[1] = phoenixError(statusSupplier, prefix);
        return periodicAlerts;
    }
}