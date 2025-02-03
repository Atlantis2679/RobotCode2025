package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class NetworkAlertsManager {
    private static class NetworkAlert {
        private final Supplier<String> messageSupplier;
        private final Alert alert;
        private final BooleanSupplier isActive;

        private NetworkAlert(Supplier<String> messageSupplier, AlertType alertType, BooleanSupplier isActive) {
            this.messageSupplier = messageSupplier;
            this.alert = new Alert(messageSupplier.get(), alertType);
            this.isActive = isActive;
        }

        private NetworkAlert(String groupName, Supplier<String> messageSupplier, AlertType alertType, BooleanSupplier isActive) {
            this.messageSupplier = messageSupplier;
            this.alert = new Alert(groupName, messageSupplier.get(), alertType);
            this.isActive = isActive;
        }

        private void update() {
            alert.setText(messageSupplier.get());
            alert.set(isActive.getAsBoolean());
        }
    }

    public static final List<NetworkAlert> alerts = new ArrayList<>();

    public static BooleanSupplier addAlert(String message, AlertType alertType, BooleanSupplier isActive) {
        return addAlert(() -> message, alertType, isActive);
    }

    public static BooleanSupplier addAlert(String groupName, String message, AlertType alertType, BooleanSupplier isActive) {
        return addAlert(groupName, () -> message, alertType, isActive);
    }

    public static BooleanSupplier addAlert(Supplier<String> message, AlertType alertType, BooleanSupplier isActive) {
        alerts.add(new NetworkAlert(message, alertType, isActive));
        return isActive;
    }

    public static BooleanSupplier addAlert(String groupName, Supplier<String> message, AlertType alertType, BooleanSupplier isActive) {
        alerts.add(new NetworkAlert(groupName, message , alertType, isActive));
        return isActive;
    }

    public static BooleanSupplier addInfoAlert(String message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(message, AlertType.kInfo, isActive);
    }

    public static BooleanSupplier addInfoAlert(String groupName, String message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(groupName, message, AlertType.kInfo, isActive);
    }

    public static BooleanSupplier addInfoAlert(Supplier<String> message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(message, AlertType.kInfo, isActive);
    }

    public static BooleanSupplier addInfoAlert(String groupName, Supplier<String> message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(groupName, message, AlertType.kInfo, isActive);
    }

    public static BooleanSupplier addWarningAlert(String message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(message, AlertType.kWarning, isActive);
    }

    public static BooleanSupplier addWarningAlert(String groupName, String message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(groupName, message, AlertType.kWarning, isActive);
    }

    public static BooleanSupplier addWarningAlert(Supplier<String> message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(message, AlertType.kWarning, isActive);
    }

    public static BooleanSupplier addWarningAlert(String groupName, Supplier<String> message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(groupName, message, AlertType.kWarning, isActive);
    }

    public static BooleanSupplier addErrorAlert(String message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(message, AlertType.kError, isActive);
    }

    public static BooleanSupplier addErrorAlert(String groupName, String message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(groupName, message, AlertType.kError, isActive);
    }

    public static BooleanSupplier addErrorAlert(Supplier<String> message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(message, AlertType.kError, isActive);
    }

    public static BooleanSupplier addErrorAlert(String groupName, Supplier<String> message, BooleanSupplier isActive) {
        return NetworkAlertsManager.addAlert(groupName, message, AlertType.kError, isActive);
    }
    public static Supplier<StatusCode> addStatusCodeAlert(String message, Supplier<StatusCode> statusCode) {
        NetworkAlertsManager.addWarningAlert(() -> message + statusCode.get().getDescription(), () -> statusCode.get().isWarning());
        NetworkAlertsManager.addErrorAlert(() -> message + statusCode.get().getDescription(), () -> statusCode.get().isError());
        return statusCode;
    }

    public static Supplier<StatusCode> addStatusCodeAlert(String groupName, String message, Supplier<StatusCode> statusCode) {
        NetworkAlertsManager.addWarningAlert(groupName, () -> message + statusCode.get().getDescription(),
            () -> statusCode.get().isWarning());
        NetworkAlertsManager.addErrorAlert(groupName, () -> message + statusCode.get().getDescription(), 
            () -> statusCode.get().isError());
        return statusCode;
    }

    public static void update() {
        for(NetworkAlert networkAlert : alerts) {
            networkAlert.update();
        }
    }

    /* HELPER METHODS: */

    public static String getREVLibErrorMessage(int value) {
        switch (value) {
            case 0: return "OK";
            case 1: return "Error";
            case 2: return "Timeout";
            case 3: return "Not Implemented";
            case 4: return "HAL Error";
            case 5: return "Can't Find Firmware";
            case 6: return "Firmware Too Old";
            case 7: return "Firmware Too New";
            case 8: return "Parameter Invalid ID";
            case 9: return "Parameter Mismatch Type";
            case 10: return "Parameter Access Mode";
            case 11: return "Parameter Invalid";
            case 12: return "Parameter Not Implemented Deprecated";
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
            case 24: return "Parameter Invalid Channel";
            case 25: return "Parameter Invalid Value";
            case 26: return "Canno't Persist Parameters While Enabled";
            default: return "Invalid";
        }
    }
}
