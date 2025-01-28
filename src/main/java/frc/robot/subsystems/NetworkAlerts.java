package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class NetworkAlerts {
    private final static List<NetworkAlerts> createdNetworksAlerts = new ArrayList<>();
    private final static String parentGroupName = "networkAlerts/";

    private final static NetworkAlerts generalNetworkAlerts = new NetworkAlerts("General Status");

    private final Map<Alert, BooleanSupplier> alerts = new HashMap<>();

    private final String groupName;

    public NetworkAlerts(String groupName) {
        this.groupName = parentGroupName + groupName;
        generalNetworkAlerts.addInfoAlert(groupName + " Is OK", this::isOK);
        generalNetworkAlerts.addWarningAlert(groupName + " Has Warnings!", this::hasWarnings);
        generalNetworkAlerts.addErrorAlert(groupName + " Has Errors!", this::hasErrors);
        createdNetworksAlerts.add(this);
    }

    public void addAlert(String message, AlertType alertType, BooleanSupplier isActive) {
        alerts.put(new Alert(groupName, message, alertType), isActive);
    }

    public void addAlert(String name , String message, AlertType alertType, BooleanSupplier isActive) {
        alerts.put(new Alert(groupName + "/" + name, message, alertType), isActive);
    }

    public void addInfoAlert(String message, BooleanSupplier isActive) {
        addAlert(message, AlertType.kInfo, isActive);
    }

    public void addInfoAlert(String name , String message, BooleanSupplier isActive) {
        addAlert(name, message, AlertType.kInfo, isActive);
    }

    public void addWarningAlert(String message, BooleanSupplier isActive) {
        addAlert(message, AlertType.kWarning, isActive);
    }

    public void addWarnignAlert(String name , String message, BooleanSupplier isActive) {
        addAlert(name, message, AlertType.kWarning, isActive);
    }

    public void addErrorAlert(String message, BooleanSupplier isActive) {
        addAlert(message, AlertType.kError, isActive);
    }

    public void addErrorAlert(String name , String message, BooleanSupplier isActive) {
        addAlert(name, message, AlertType.kError, isActive);
    }

    public void addSwitchAlert(String activeMessage, String unactiveMessage, AlertType activeAlertType, AlertType unactiveAlertType, BooleanSupplier isActive) {
        addAlert(activeMessage, activeAlertType, isActive);
        addAlert(unactiveMessage, unactiveAlertType, () -> !isActive.getAsBoolean());
    }

    public void addSwitchAlert(String name, String activeMessage, String unactiveMessage, AlertType activeAlertType, AlertType unactiveAlertType, BooleanSupplier isActive) {
        addAlert(activeMessage, activeAlertType, isActive);
        addAlert(unactiveMessage, unactiveAlertType, () -> !isActive.getAsBoolean());
    }

    public void addInfoWarningSwitchAlert(String infoMessage, String warningMessage, BooleanSupplier isWarningActive) {
        addSwitchAlert(warningMessage, infoMessage, AlertType.kWarning, AlertType.kInfo, isWarningActive);
    }

    public void addInfoWarningSwitchAlert(String name, String infoMessage, String warningMessage, BooleanSupplier isWarningActive) {
        addSwitchAlert(name, warningMessage, infoMessage, AlertType.kWarning, AlertType.kInfo, isWarningActive);
    }

    public void addInfoErrorSwitchAlert(String infoMessage, String errorMessage, BooleanSupplier isErrorActive) {
        addSwitchAlert(errorMessage, infoMessage, AlertType.kWarning, AlertType.kInfo, isErrorActive);
    }

    public void addInfoErrorSwitchAlert(String name, String infoMessage, String errorMessage, BooleanSupplier isErrorActive) {
        addSwitchAlert(name, errorMessage, infoMessage, AlertType.kWarning, AlertType.kInfo, isErrorActive);
    }


    public void addWarningErrorSwitchAlert(String warningMessage, String errorMessage, BooleanSupplier isErrorActive) {
        addSwitchAlert(errorMessage, warningMessage, AlertType.kError, AlertType.kWarning, isErrorActive);
    }

    public void addWarningErrorSwitchAlert(String name, String warningMessage, String errorMessage, BooleanSupplier isErrorActive) {
        addSwitchAlert(name, errorMessage, warningMessage, AlertType.kError, AlertType.kWarning, isErrorActive);
    }

    public void addStatusAlert(String message, BooleanSupplier isInfoActive, BooleanSupplier isWarningActive, BooleanSupplier isErrorActive) {
        addInfoAlert(message, isInfoActive);
        addWarningAlert(message, isWarningActive);
        addErrorAlert(message, isErrorActive);
    }

    public void addStatusAlert(String name, String message, BooleanSupplier isInfoActive, BooleanSupplier isWarningActive, BooleanSupplier isErrorActive) {
        addInfoAlert(name, message, isInfoActive);
        addWarnignAlert(name,message, isWarningActive);
        addErrorAlert(name,message, isErrorActive);
    }

    public static void update() {
        for (NetworkAlerts networkAlerts : createdNetworksAlerts) {
            networkAlerts.alerts.forEach((alert, isActive) -> {
                alert.set(isActive.getAsBoolean());
            });
        }
    }

    public boolean hasErrors() {
        for (Alert alert : alerts.keySet()) {
            if(alert.get() && alert.getType() == AlertType.kError)
                return true;
        } return false;
    }

    public boolean hasWarnings() {
        for (Alert alert : alerts.keySet()) {
            if(alert.get() && alert.getType() == AlertType.kWarning)
                return true;
        } return false;
    }

    public boolean isOK() {
        return !(hasErrors() || hasWarnings());
    }
}
