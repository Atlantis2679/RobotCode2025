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

        private String getMessage() {
            return messageSupplier.get();
        }

        private Alert getAlert() {
            return alert;
        }

        private boolean getIsActive() {
            return isActive.getAsBoolean();
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
            networkAlert.getAlert().setText(networkAlert.getMessage());
            networkAlert.getAlert().set(networkAlert.getIsActive());
        }
    }
}
