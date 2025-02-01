package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class NetworkAlertsManager {
    public static final Map<Alert, BooleanSupplier> alerts = new HashMap<>();

    public static BooleanSupplier addAlert(String message, AlertType alertType, BooleanSupplier isActive) {
        alerts.put(new Alert(message, alertType), isActive);
        return isActive;
    }

    public static BooleanSupplier addAlert(String group, String message, AlertType alertType, BooleanSupplier isActive) {
        alerts.put(new Alert(group, message, alertType), isActive);
        return isActive;
    }

    public static BooleanSupplier addInfoAlert(String message, BooleanSupplier isActive) {
        NetworkAlertsManager.addAlert(message, AlertType.kInfo, isActive);
        return isActive;
    }

    public static BooleanSupplier addInfoAlert(String group, String message, BooleanSupplier isActive) {
        NetworkAlertsManager.addAlert(group, message, AlertType.kInfo, isActive);
        return isActive;
    }

    public static BooleanSupplier addWarningAlert(String message, BooleanSupplier isActive) {
        NetworkAlertsManager.addAlert(message, AlertType.kWarning, isActive);
        return isActive;
    }

    public static BooleanSupplier addWarningAlert(String group, String message, BooleanSupplier isActive) {
        NetworkAlertsManager.addAlert(group, message, AlertType.kWarning, isActive);
        return isActive;
    }

    public static BooleanSupplier addErrorAlert(String message, BooleanSupplier isActive) {
        NetworkAlertsManager.addAlert(message, AlertType.kError, isActive);
        return isActive;
    }

    public static BooleanSupplier addErrorAlert(String group, String message, BooleanSupplier isActive) {
        NetworkAlertsManager.addAlert(group, message, AlertType.kError, isActive);
        return isActive;
    }

    public static StatusCode addStatusCodeAlert(String message, StatusCode statusCode) {
        NetworkAlertsManager.addWarningAlert(message, statusCode::isWarning);
        NetworkAlertsManager.addErrorAlert(message, statusCode::isError);
        return statusCode;
    }

    public static void update() {
        alerts.forEach((alert, isActive) -> {
            alert.set(isActive.getAsBoolean());
        });
    }
}
