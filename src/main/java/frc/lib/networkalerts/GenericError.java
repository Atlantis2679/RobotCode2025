package frc.lib.networkalerts;

import edu.wpi.first.wpilibj.Alert.AlertType;

public record GenericError(String message, String alertGroup, String errorType, String details, int errorCode, boolean isActive, AlertType alertType) {}
