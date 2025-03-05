package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class NetworkAlertsManager {
    private static class NetworkPeriodicAlert {
        private final Supplier<String> messageSupplier;
        private final Alert alert;
        private final BooleanSupplier isActive;

        private NetworkPeriodicAlert(Supplier<String> messageSupplier, AlertType alertType, BooleanSupplier isActive) {
            this.messageSupplier = messageSupplier;
            this.alert = new Alert(messageSupplier.get(), alertType);
            this.isActive = isActive;
        }

        private NetworkPeriodicAlert(String groupName, Supplier<String> messageSupplier, AlertType alertType, BooleanSupplier isActive) {
            this.messageSupplier = messageSupplier;
            this.alert = new Alert(groupName, messageSupplier.get(), alertType);
            this.isActive = isActive;
        }

        private void update() {
            alert.setText(messageSupplier.get());
            alert.set(isActive.getAsBoolean());
        }
    }

    public static final List<NetworkPeriodicAlert> alerts = new ArrayList<>();

    public static BooleanSupplier addAlert(String message, AlertType alertType, BooleanSupplier isActive) {
        return addAlert(() -> message, alertType, isActive);
    }

    public static BooleanSupplier addAlert(Supplier<String> message, AlertType alertType, BooleanSupplier isActive) {
        alerts.add(new NetworkPeriodicAlert(message, alertType, isActive));
        return isActive;
    }

    public static BooleanSupplier addInfoAlert(String message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kInfo, isActive);
    }

    public static BooleanSupplier addInfoAlert(Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kInfo, isActive);
    }

    public static BooleanSupplier addWarningAlert(String message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kWarning, isActive);
    }

    public static BooleanSupplier addWarningAlert(Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kWarning, isActive);
    }

    public static BooleanSupplier addErrorAlert(String message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kError, isActive);
    }

    public static BooleanSupplier addErrorAlert(Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kError, isActive);
    }

    public static Supplier<StatusCode> addStatusCodeAlert(String message, Supplier<StatusCode> statusCode) {
        addWarningAlert(() -> message + statusCode.get().getDescription(), () -> statusCode.get().isWarning());
        addErrorAlert(() -> message + statusCode.get().getDescription(), () -> statusCode.get().isError());
        return statusCode;
    }

    public static void addSparkMotorAlert(String message, Supplier<Faults> faults, Supplier<Warnings> warnings) {
        addErrorAlert(() -> message + getSparkFaultsMessage(faults.get()), () -> faults.get().rawBits != 0);
        addWarningAlert(() -> message + getSprakWarningsMessage(warnings.get()),
        () -> warnings.get().rawBits != 0);
    }

    public static Supplier<REVLibError> addRevLibErrorAlert(String message, Supplier<REVLibError> error) {
        addErrorAlert(() -> getREVLibErrorMessage(error.get()), () -> error.get().value != 0);
        return error;
    }

    /** Update all the periodic alerts. This should be called every robot periodic. */
    public static void update() {
        for(NetworkPeriodicAlert networkAlert : alerts) {
            networkAlert.update();
        }
    }

    /* HELPER METHODS: */

    public static String getREVLibErrorMessage(REVLibError error) {
        switch (error.value) {
            case 0: return "OK";
            case 1: return "Error";
            case 2: return "Timeout";
            case 3: return "Not Implemented";
            case 4: return "HAL Error";
            case 5: return "Can't Find Firmware";
            case 6: return "Firmware Too Old";
            case 7: return "Firmware Too New";
            case 8: return "Param Invalid ID";
            case 9: return "Param Mismatch Type";
            case 10: return "Param Access Mode";
            case 11: return "Param Invalid";
            case 12: return "Param Not Implemented Deprecated";
            case 13: return "Follow Config Mismatch";
            case 14: return "Invalid";
            case 15: return "Setpoint Out Of Range";
            case 16: return "Unknown";
            case 17: return "CAN Disconnected";
            case 18: return "Duplicate CAN Id";
            case 19: return "Invalid CAN Id";
            case 20: return "Spark Max Data Port Already Configured Differently";
            case 21: return "Spark Flex Brushed Without Dock";
            case 22: return "Invalid Brushless Encoder Configuration";
            case 23: return "Feedback Sensor Incompatible With Data Port Config";
            case 24: return "Param Invalid Channel";
            case 25: return "Param Invalid Value";
            case 26: return "Canno't Persist Parameters While Enabled";
            default: return "Invalid";
        }
    }

    public static String getSparkFaultsMessage(Faults faults) {
        return (faults.can ? "can, " : "") + (faults.escEeprom ? "escEeprom, " : "")
            + (faults.firmware ? "firmware, " : "" + faults) + (faults.gateDriver ? "gateDriver, " : "")
            + (faults.motorType ? "motorType, " : "") + (faults.other ? "other, " : "") + (faults.sensor ? "sensor, " : "")
            + (faults.temperature ? "temperature, " : "");
    }

    public static String getSprakWarningsMessage(Warnings warnings) {
        return (warnings.brownout ? "brownout, " : "") + (warnings.escEeprom ? "escEeprom, " : "")
            + (warnings.extEeprom ? "extEeprom, " : "") + (warnings.hasReset ? "hasReset, " : "") + (warnings.other ? "other, " : "")
            + (warnings.overcurrent ? "overcurrent, " : "") + (warnings.sensor ? "sensor, ": "") + (warnings.stall ? "stall, " : "");
    }
}