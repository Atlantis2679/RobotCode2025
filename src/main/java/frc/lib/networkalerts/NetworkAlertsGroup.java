package frc.lib.networkalerts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert.AlertType;

public class NetworkAlertsGroup {
    public static final NetworkAlertsGroup defaultInstance = new NetworkAlertsGroup("NetworkAlerts");
    private final String groupName;

    public NetworkAlertsGroup(String groupName) {
        this.groupName = groupName;
    }

    public NetworkAlertsGroup getSubGroup(String subGroupName) {
        return new NetworkAlertsGroup(groupName + "/" + subGroupName);
    }

    public BooleanSupplier addNetworkPeriodicAlert(NetworkPeriodicAlert periodicAlert) {
        return NetworkAlertsManager.addNetworkPeriodicAlert(periodicAlert);
    }

    public BooleanSupplier[] addNetworkPeriodicAlertArray(NetworkPeriodicAlert[] periodicAlerts) {
        return NetworkAlertsManager.addNetworkPeriodicAlertArray(periodicAlerts); 
    }

    public BooleanSupplier addAlert(Supplier<String> message, AlertType alertType, BooleanSupplier isActive) {
        NetworkPeriodicAlert periodicAlert = new NetworkPeriodicAlert(this, message, alertType, isActive);
        return NetworkAlertsManager.addNetworkPeriodicAlert(periodicAlert);
    }

    public BooleanSupplier addInfoAlert(Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kInfo, isActive);
    }

    public BooleanSupplier addWarningAlert(Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kWarning, isActive);
    }

    public BooleanSupplier addErrorAlert(Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kError, isActive);
    }

    public String getName() {
        return groupName;
    }
}