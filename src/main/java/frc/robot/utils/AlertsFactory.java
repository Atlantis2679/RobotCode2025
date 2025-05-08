package frc.robot.utils;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import frc.lib.networkalerts.NetworkAlertsGroup;

public final class AlertsFactory {
    public static BooleanSupplier[] revMotor(NetworkAlertsGroup group ,Supplier<REVLibError> configErrorSupplier,
            Supplier<Warnings> warningSupplier, Supplier<Faults> errorSupplier, String prefix) {
        BooleanSupplier[] alertsActivations = new BooleanSupplier[3];
        alertsActivations[0] = revError(group, configErrorSupplier, prefix);
        alertsActivations[1] = sparkMaxWarning(group, warningSupplier, prefix);
        alertsActivations[2] = sparkMaxError(group, errorSupplier, prefix);
        return alertsActivations;
    }

    public static BooleanSupplier revError(NetworkAlertsGroup group, Supplier<REVLibError> errorSupplier, String prefix) {
        return group.addErrorAlert(() -> prefix + ": " + errorSupplier.get().name(), () -> errorSupplier.get().value != 0);
    }

    public static BooleanSupplier sparkMaxWarning(NetworkAlertsGroup group, Supplier<Warnings> warningSupplier, String prefix) {
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
        return group.addWarningAlert(messageSupplier, () -> warningSupplier.get().rawBits != 0);
    }

    public static BooleanSupplier sparkMaxError(NetworkAlertsGroup group, Supplier<Faults> errorSupplier, String prefix) {
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
        return group.addErrorAlert(messageSupplier, () -> errorSupplier.get().rawBits != 0);
    }

    public static BooleanSupplier phoenixWarning(NetworkAlertsGroup group, Supplier<StatusCode> statusSupplier, String prefix) {
        return group.addWarningAlert(() -> prefix + ": " + statusSupplier.get().getName(), () -> statusSupplier.get().isWarning());
    }

    public static BooleanSupplier phoenixError(NetworkAlertsGroup group, Supplier<StatusCode> statusSupplier, String prefix) {
        return group.addErrorAlert(() -> prefix + ": " + statusSupplier.get().getName(), () -> statusSupplier.get().isError());
    }

    public static BooleanSupplier[] phoenixMotor(NetworkAlertsGroup group, Supplier<StatusCode> statusSupplier, String prefix) {
        BooleanSupplier[] alertsActivations = new BooleanSupplier[2];
        alertsActivations[0] = phoenixWarning(group, statusSupplier, prefix);
        alertsActivations[1] = phoenixError(group, statusSupplier, prefix);
        return alertsActivations;
    }
}