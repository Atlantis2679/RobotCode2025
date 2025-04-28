package frc.lib.networkalerts;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkBase.Faults;

import edu.wpi.first.wpilibj.Alert.AlertType;

public class GenericErrorGeneratorLib {
    public static GenericError revError(REVLibError error, String prefix, String motorName, String alertGroup) {
        return new GenericError(prefix + ": " + motorName + ": " + error.name(), alertGroup, "RevLibError", "none", error.value, error.value != 0, AlertType.kError);
    }

    public static GenericError revError(REVLibError error, String prefix, String motorName) {
        return revError(error, prefix, null);
    }

    public static GenericError sparkMaxWarning(Warnings warning, String prefix, String motorName, String alertGroup) {
        String message = "";
        if (warning.brownout) message += "brownout, ";
        if (warning.escEeprom) message += "escEeprom, ";
        if (warning.extEeprom) message += "extEeprom, ";
        if (warning.hasReset) message += "hasReset, ";
        if (warning.other) message += "other, ";
        if (warning.overcurrent) message += "overcurrent, ";
        if (warning.sensor) message += "sensor, ";
        if (warning.stall) message += "stall, ";
        return new GenericError(prefix + ": " + motorName + ": " + message, alertGroup, "SparkMaxWarning", "none", warning.rawBits, warning.rawBits != 0, AlertType.kWarning);
    }

    public static GenericError sparkMaxWarning(Warnings warning, String prefix, String motorName) {
        return sparkMaxWarning(warning, prefix, motorName, null);
    }

    public static GenericError sparkMaxError(Faults error, String prefix, String motorName, String alertGroup) {
        String message = "";
        if (error.can) message += "can, ";
        if (error.escEeprom) message += "escEeprom, ";
        if (error.firmware) message += "firmware, ";
        if (error.gateDriver) message += "gateDriver, ";
        if (error.motorType) message += "motorType, ";
        if (error.other) message += "other, ";
        if (error.sensor) message += "sensor, ";
        if (error.temperature) message += "temperature, ";
        return new GenericError(prefix + ": " + motorName + ": " + message, alertGroup, "SparkMaxError", "none", error.rawBits, error.rawBits != 0, AlertType.kError);
    }

    public static GenericError sparkMaxError(Faults error, String prefix, String motorName) {
        return sparkMaxError(error, prefix, null);
    }

    public static GenericError phoenixError(StatusCode status, String prefix, String motorName, String alertGroup) {
        return new GenericError(prefix + ": " + motorName + ": " + status.getName(), alertGroup, "PhoenixError", status.getDescription(), status.value, !status.isOK(), status.isWarning() ? AlertType.kWarning : AlertType.kError);
    }

    public static GenericError phoenixError(StatusCode status, String prefix, String motorName) {
        return phoenixError(status, prefix, motorName, null);
    }

    public static GenericError deviceDisconnectedError(boolean isConnected, String prefix, String deviceName, String alertGroup) {
        return new GenericError(prefix + ": " + deviceName + " Is Disconnected!", alertGroup, "DeviceDisconnected", "none", isConnected ? 0 : 1, !isConnected, AlertType.kError);
    }

    public static GenericError deviceDisconnectedError(boolean isConnected, String prefix, String deviceName) {
        return deviceDisconnectedError(isConnected, prefix, deviceName, null);
    }

    public static GenericError deviceOverheat(int temperature, int cap, String prefix, String deviceName, String alertGroup) {
        return new GenericError(prefix + ": " + deviceName + " Overheat (" + temperature + "C)!", alertGroup, "DeviceOverheat", "none", temperature >= cap ? 0 : 1, temperature >= cap, AlertType.kWarning);
    }

    public static GenericError deviceOverheat(int temperature, int cap, String prefix, String deviceName) {
        return deviceOverheat(temperature, cap, prefix, deviceName, null);
    }
}
