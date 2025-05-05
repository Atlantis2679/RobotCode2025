package frc.robot.utils;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.networkalerts.NetworkPeriodicAlert;

public final class AlertsFactory {
    public static NetworkPeriodicAlert revError(REVLibError error, String prefix, String motorName, String alertGroup) {
        return new NetworkPeriodicAlert(alertGroup, () -> prefix + ": " + motorName + ": " + error.name(), AlertType.kError, () -> error.value != 0);
    }

    public static NetworkPeriodicAlert revError(REVLibError error, String prefix, String motorName) {
        return revError(error, prefix, null);
    }

    public static NetworkPeriodicAlert sparkMaxWarning(Supplier<Warnings> warningSupplier, String prefix, String motorName, String alertGroup) {
        Supplier<String> messageSupplier = () -> {
          String message = prefix + ": " + motorName + ": ";
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
        return new NetworkPeriodicAlert(alertGroup, messageSupplier, AlertType.kWarning, () -> warningSupplier.get().rawBits != 0);
    }

    public static NetworkPeriodicAlert sparkMaxWarning(Supplier<Warnings> warningSupplier, String prefix, String motorName) {
        return sparkMaxWarning(warningSupplier, prefix, motorName, null);
    }

    public static NetworkPeriodicAlert sparkMaxError(Supplier<Faults> errorSupplier, String prefix, String motorName, String alertGroup) {
        Supplier<String> messageSupplier = () -> {
          String message = prefix + ": " + motorName + ": ";
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
        return new NetworkPeriodicAlert(alertGroup, messageSupplier, AlertType.kError, () -> errorSupplier.get().rawBits != 0);
    }

    public static NetworkPeriodicAlert sparkMaxError(Supplier<Faults> errorSupplier, String prefix, String motorName) {
        return sparkMaxError(errorSupplier, prefix, motorName, null);
    }

    public static NetworkPeriodicAlert phoenixError(Supplier<StatusCode> statusSupplier, String prefix, String motorName, String alertGroup) {
        return new NetworkPeriodicAlert(alertGroup, () -> prefix + " " + motorName + ": " + statusSupplier.get().getName(), AlertType.kWarning , () -> statusSupplier.get().isWarning());
    }

    public static NetworkPeriodicAlert phoenixError(Supplier<StatusCode> statusSupplier, String prefix, String motorName) {
        return phoenixError(statusSupplier, prefix, motorName, null);
    }
    
    // public static GenericError deviceDisconnectedError(boolean isConnected, String prefix, String deviceName, String alertGroup) {
    //     return new GenericError(prefix + ": " + deviceName + " Is Disconnected!", alertGroup, "DeviceDisconnected", "none", isConnected ? 0 : 1, !isConnected, AlertType.kError);
    // }

    // public static GenericError deviceDisconnectedError(boolean isConnected, String prefix, String deviceName) {
    //     return deviceDisconnectedError(isConnected, prefix, deviceName, null);
    // }

    // public static GenericError deviceOverheat(int temperature, int cap, String prefix, String deviceName, String alertGroup) {
    //     return new GenericError(prefix + ": " + deviceName + " Overheat (" + temperature + "C)!", alertGroup, "DeviceOverheat", "none", temperature >= cap ? 0 : 1, temperature >= cap, AlertType.kWarning);
    // }

    // public static GenericError deviceOverheat(int temperature, int cap, String prefix, String deviceName) {
    //     return deviceOverheat(temperature, cap, prefix, deviceName, null);
    // }
}