package frc.lib.networkalerts;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert.AlertType;

public class NetworkAlertsManager {
    private static final List<NetworkPeriodicAlert> alerts = new ArrayList<>();
    
    /** Update all the periodic alerts. This should be called every robot periodic. */
    public static void update() {
        for(NetworkPeriodicAlert networkAlert : alerts) {
            networkAlert.update();
        }
    }

    public static BooleanSupplier addNetworkPeriodicAlert(NetworkPeriodicAlert periodicAlert) {
        alerts.add(periodicAlert);
        return periodicAlert::getIsActive;
    }

    public static BooleanSupplier addAlert(Supplier<String> message, AlertType alertType, BooleanSupplier isActive) {
        return addAlert(null, message, alertType, isActive);
    }

    public static BooleanSupplier addAlert(String groupName, Supplier<String> message, AlertType alertType, BooleanSupplier isActive) {
        alerts.add(new NetworkPeriodicAlert(groupName, message, alertType, isActive));
        return isActive;
    }

    public static BooleanSupplier addInfoAlert(Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kInfo, isActive);
    }

    public static BooleanSupplier addInfoAlert(String groupName, Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(groupName, message, AlertType.kInfo, isActive);
    }

    public static BooleanSupplier addWarningAlert(Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kWarning, isActive);
    }

    public static BooleanSupplier addWarningAlert(String groupName, Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(groupName, message, AlertType.kWarning, isActive);
    }

    public static BooleanSupplier addErrorAlert(Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(message, AlertType.kError, isActive);
    }

    public static BooleanSupplier addErrorAlert(String groupName, Supplier<String> message, BooleanSupplier isActive) {
        return addAlert(groupName, message, AlertType.kError, isActive);
    }

}