package frc.lib.networkalerts;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

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

    public static BooleanSupplier[] addNetworkPeriodicAlertArray(NetworkPeriodicAlert[] periodicAlerts) {
        BooleanSupplier[] activeSuppliers = new BooleanSupplier[periodicAlerts.length];
        for (int i = 0; i < activeSuppliers.length; i++) {
            activeSuppliers[i] = addNetworkPeriodicAlert(periodicAlerts[i]);
        }
        return activeSuppliers;
    }
}