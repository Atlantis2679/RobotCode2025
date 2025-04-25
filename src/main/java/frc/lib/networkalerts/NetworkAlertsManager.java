package frc.lib.networkalerts;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class NetworkAlertsManager {
    private static final List<NetworkPeriodicAlert> alerts = new ArrayList<>();
    
    /** Update all the periodic alerts. This should be called every robot periodic. */
    public static void update() {
        for(NetworkPeriodicAlert networkAlert : alerts) {
            networkAlert.update();
        }
    }

    public static BooleanSupplier addAlert(Supplier<String> message, AlertType alertType, BooleanSupplier isActive) {
        return addAlert(message, () -> alertType, isActive);
    }

    public static BooleanSupplier addAlert(String groupName, Supplier<String> message, AlertType alertType, BooleanSupplier isActive) {
        return addAlert(() -> groupName, message, () -> alertType, isActive);
    }

    public static BooleanSupplier addAlert(Supplier<String> message, Supplier<AlertType> alertType, BooleanSupplier isActive) {
        alerts.add(new NetworkPeriodicAlert(message, alertType, isActive));
        return isActive;
    }

    public static BooleanSupplier addAlert(Supplier<String> groupName, Supplier<String> message, Supplier<AlertType> alertType, BooleanSupplier isActive) {
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

    public static BooleanSupplier addGenericError(Supplier<GenericError> error) {
        if (error.get().alertGroup() != null)
            return addAlert(error.get().alertGroup(), () -> error.get().message(), error.get().alertType(), () -> error.get().isActive());
        else
            return addAlert(() -> error.get().message(), error.get().alertType(), () -> error.get().isActive());
    }

    private static class NetworkPeriodicAlert {
        private static final String defaultGroupName = "NetworkAlerts";
        private final Supplier<String> groupSupplier;
        private final Supplier<String> messageSupplier;
        private String currentGruopName;
        private final Supplier<AlertType> alertTypeSupplier;
        private Alert alert;
        private final BooleanSupplier isActive;

        private NetworkPeriodicAlert(Supplier<String> messageSupplier, Supplier<AlertType> alertTypeSupplier, BooleanSupplier isActive) {
            this(() -> defaultGroupName, messageSupplier, alertTypeSupplier, isActive);
        }

        private NetworkPeriodicAlert(Supplier<String> groupSupplier, Supplier<String> messageSupplier, Supplier<AlertType> alertTypeSupplier, BooleanSupplier isActive) {
            this.groupSupplier = groupSupplier;
            this.currentGruopName = groupSupplier.get();
            this.messageSupplier = messageSupplier;
            this.alertTypeSupplier = alertTypeSupplier;
            this.alert = new Alert(currentGruopName, messageSupplier.get(), alertTypeSupplier.get());
            this.isActive = isActive;
        }

        private void update() {
            if (alert.getType() != alertTypeSupplier.get() || currentGruopName != groupSupplier.get()) {
                this.currentGruopName = groupSupplier.get();
                this.alert = new Alert(currentGruopName, messageSupplier.get(), alertTypeSupplier.get());
            }
            alert.setText(messageSupplier.get());
            alert.set(isActive.getAsBoolean());
        }
    }
}