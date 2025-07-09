package frc.robot.utils;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public final class AlertsFactory {
    /*
       This will add all the different alert that a rev motor can generate, and return an array of their activations
       (The order of the array is the order of the parameters)
    */
    public static BooleanSupplier[] revMotor(PeriodicAlertsGroup group, String prefix, Supplier<REVLibError> configErrorSupplier,
            Supplier<Warnings> warningSupplier, Supplier<Faults> errorSupplier) {
        BooleanSupplier[] alertsActivations = new BooleanSupplier[3];
        alertsActivations[0] = revError(group, prefix, configErrorSupplier);
        alertsActivations[1] = sparkMaxWarning(group, prefix, warningSupplier);
        alertsActivations[2] = sparkMaxError(group, prefix, errorSupplier);
        return alertsActivations;
    }

    /* This will add a `REVLibError` alert and return the alert's activation */
    public static BooleanSupplier revError(PeriodicAlertsGroup group, String prefix, Supplier<REVLibError> errorSupplier) {
        return group.addErrorAlert(() -> prefix + ": " + errorSupplier.get().name(), () -> errorSupplier.get().value != 0);
    }


    /* This will add a spark max (rev) warning alert and return the alert's activation */
    public static BooleanSupplier sparkMaxWarning(PeriodicAlertsGroup group, String prefix, Supplier<Warnings> warningSupplier) {
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

    /* This will add a spark max (rev) error alert and return the alert's activation */
    public static BooleanSupplier sparkMaxError(PeriodicAlertsGroup group, String prefix, Supplier<Faults> errorSupplier) {
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

    /* This will add a warning phenix alert and return alert's activation */
    public static BooleanSupplier phoenixWarning(PeriodicAlertsGroup group, String prefix, Supplier<StatusCode> statusSupplier) {
        return group.addWarningAlert(() -> prefix + ": " + statusSupplier.get().getName(), () -> statusSupplier.get().isWarning());
    }

    /* This will add an error phenix alert and return alert's activation */
    public static BooleanSupplier phoenixError(PeriodicAlertsGroup group, String prefix, Supplier<StatusCode> statusSupplier) {
        return group.addErrorAlert(() -> prefix + ": " + statusSupplier.get().getName(), () -> statusSupplier.get().isError());
    }

    /*
       This will add all the different alert that a phoenix motor can generate, and return an array of their activations
       (The order of the array is the order of the parameters)
    */
    public static BooleanSupplier[] phoenixMotor(PeriodicAlertsGroup group, String prefix, Supplier<StatusCode> statusSupplier) {
        BooleanSupplier[] alertsActivations = new BooleanSupplier[2];
        alertsActivations[0] = phoenixWarning(group, prefix, statusSupplier);
        alertsActivations[1] = phoenixError(group, prefix, statusSupplier);
        return alertsActivations;
    }
}