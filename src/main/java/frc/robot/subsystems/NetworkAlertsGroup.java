package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class NetworkAlertsGroup {
    private final static List<NetworkAlertsGroup> createdGroups = new ArrayList<>();
    private final static String parentFolder = "networkAlerts";

    private final static NetworkAlertsGroup robotStatusGroup = new NetworkAlertsGroup(
        parentFolder, "Robot Status", false);

    private final Map<Alert, BooleanSupplier> alerts = new HashMap<>();

    private final String groupName;
    private final String groupFolder;

    private boolean hasWarnings = false;
    private boolean hasErrors = false;

    private NetworkAlertsGroup(String parentGroupFolder, String groupName, boolean addToRobotStatusGroup) {
        this.groupName = groupName;
        this.groupFolder = parentGroupFolder + "/" + groupName;
        if (addToRobotStatusGroup) {
            robotStatusGroup.addInfoAlert(groupName + " Is OK", this::getIsOK);
            robotStatusGroup.addWarningAlert(groupName + " Has Warnings!", this::getHasWarnings);
            robotStatusGroup.addErrorAlert(groupName + " Has Errors!", this::getHasErrors);
            createdGroups.add(this);
        }
    }
    
    public NetworkAlertsGroup(String groupName) {
        this(parentFolder, groupName, true);
    }

    public String getGroupName() {
        return groupName;
    }

    public String getGroupFolder() {
        return groupFolder;
    }

    public boolean getHasErrors() {
        return hasErrors;
    }

    public boolean getHasWarnings() {
        return hasWarnings && !hasErrors;
    }

    public boolean getIsOK() {
        return !(hasErrors || hasWarnings);
    }

    /* A sub group is recorded seperatly from it's parent in the robot status alerts */
    public NetworkAlertsGroup getSubGroup(String subGroupName) {
        return new NetworkAlertsGroup(groupFolder, subGroupName, true);
    }
    
    public void addAlert(String message, AlertType alertType, BooleanSupplier isActive) {
        alerts.put(new Alert(groupFolder + "/General", message, alertType), isActive);
    }

    /* A subGroupName is a way to create a group that isn't ment to be shown seperate from his
     * parent group. For example, each module may be in a group named "Modules",
     * but it can contain more than one alert (each motor has it's own status alert) */
 
    public void addAlert(String subGroupName, String message, AlertType alertType, BooleanSupplier isActive) {
        alerts.put(new Alert(groupFolder + "/" + subGroupName, message, alertType), isActive);
    }

    public void addInfoAlert(String message, BooleanSupplier isActive) {
        addAlert(message, AlertType.kInfo, isActive);
    }

    public void addInfoAlert(String subGroupName, String message, BooleanSupplier isActive) {
        addAlert(subGroupName, message, AlertType.kInfo, isActive);
    }

    public void addWarningAlert(String message, BooleanSupplier isActive) {
        addAlert(message, AlertType.kWarning, isActive);
    }
    
    public void addWarningAlert(String subGroupName, String message, BooleanSupplier isActive) {
        addAlert(subGroupName, message, AlertType.kWarning, isActive);
    }

    public void addErrorAlert(String message, BooleanSupplier isActive) {
        addAlert(message, AlertType.kError, isActive);
    }
    
    public void addErrorAlert(String subGroupName, String message, BooleanSupplier isActive) {
        addAlert(subGroupName, message, AlertType.kError, isActive);
    }

    public void addSwitchAlert(String activeMessage, String unactiveMessage, AlertType activeAlertType, 
                               AlertType unactiveAlertType, BooleanSupplier isActive) {
        addAlert(activeMessage, activeAlertType, isActive);
        addAlert(unactiveMessage, unactiveAlertType, () -> !isActive.getAsBoolean());
    }
    
    public void addSwitchAlert(String subGroupName, String activeMessage, String unactiveMessage, 
                               AlertType activeAlertType, AlertType unactiveAlertType, BooleanSupplier isActive) {
        addAlert(subGroupName, activeMessage, activeAlertType, isActive);
        addAlert(subGroupName, unactiveMessage, unactiveAlertType, () -> !isActive.getAsBoolean());
    }

    public void addInfoInfoSwitchAlert(String firstInfoMessage, String secondInfoMessage, 
                                       BooleanSupplier isFirstActive) {
        addSwitchAlert(firstInfoMessage, secondInfoMessage, AlertType.kInfo, AlertType.kInfo, isFirstActive);
    }

    public void addInfoInfoSwitchAlert(String subGroupName, String firstInfoMessage, String secondInfoMessage, 
                                       BooleanSupplier isFirstActive) {
        addSwitchAlert(subGroupName, firstInfoMessage, secondInfoMessage, AlertType.kInfo,
                       AlertType.kInfo, isFirstActive);
    }

    public void addInfoWarningSwitchAlert(String infoMessage, String warningMessage, 
                                          BooleanSupplier isInfoActive) {
        addSwitchAlert(infoMessage, warningMessage, AlertType.kInfo, AlertType.kWarning, isInfoActive);
    }

    public void addInfoWarningSwitchAlert(String subGroupName, String infoMessage, String warningMessage, 
                                          BooleanSupplier isInfoActive) {
        addSwitchAlert(subGroupName, infoMessage, warningMessage, AlertType.kInfo, AlertType.kWarning, 
                       isInfoActive);
    }

    public void addInfoErrorSwitchAlert(String infoMessage, String errorMessage,
                                        BooleanSupplier isInfoActive) {
        addSwitchAlert(infoMessage, errorMessage, AlertType.kInfo, AlertType.kError, isInfoActive);
    }

    public void addInfoErrorSwitchAlert(String subGroupName, String infoMessage, String errorMessage, 
                                        BooleanSupplier isInfoActive) {
        addSwitchAlert(subGroupName, infoMessage, errorMessage, AlertType.kInfo, AlertType.kError, 
                       isInfoActive);
    }

    public void addWarningErrorSwitchAlert(String warningMessage, String errorMessage, 
                                           BooleanSupplier isWarningActive) {
        addSwitchAlert(warningMessage, errorMessage, AlertType.kWarning, AlertType.kError, isWarningActive);
    }

    public void addWarningErrorSwitchAlert(String subGroupName, String warningMessage, String errorMessage, 
                                           BooleanSupplier isWarningActive) {
        addSwitchAlert(subGroupName, warningMessage, errorMessage, AlertType.kWarning, 
                       AlertType.kError, isWarningActive);
    }

    public void addStatusAlert(String message, BooleanSupplier isInfoActive, 
                               BooleanSupplier isWarningActive, BooleanSupplier isErrorActive) {
        addInfoAlert(message, isInfoActive);
        addWarningAlert(message, isWarningActive);
        addErrorAlert(message, isErrorActive);
    }

    public void addStatusAlert(String subGroupName, String message, BooleanSupplier isInfoActive, 
                               BooleanSupplier isWarningActive, BooleanSupplier isErrorActive) {
        addInfoAlert(subGroupName, message, isInfoActive);
        addWarningAlert(subGroupName, message, isWarningActive);
        addErrorAlert(subGroupName, message, isErrorActive);
    }

    public static void update() {
        for (NetworkAlertsGroup networkAlertsGroup : createdGroups) {
            networkAlertsGroup.alerts.forEach((alert, isActive) -> {
                alert.set(isActive.getAsBoolean());
                if(alert.get() && alert.getType() == AlertType.kWarning)
                    networkAlertsGroup.hasWarnings = true;
                if(alert.get() && alert.getType() == AlertType.kError)
                    networkAlertsGroup.hasErrors = true;
            });
        }
        /* Done seperatly in order that the robot status will be synchronized with */
        /* the hasWarnings and hasErrors.                                          */
        robotStatusGroup.alerts.forEach((alert, isActive) -> {
                alert.set(isActive.getAsBoolean());
        });
    }

    /* static methods to make the alerts data more accesible */

    public static NetworkAlertsGroup getGroupByName(String groupName) {
        for(NetworkAlertsGroup networkAlertsGroup : createdGroups) {
            if(networkAlertsGroup.getGroupName().equals(groupName))
                return networkAlertsGroup;
        }
        if(robotStatusGroup.getGroupName().equals(groupName))
           return robotStatusGroup;
        return null;
    }
}
