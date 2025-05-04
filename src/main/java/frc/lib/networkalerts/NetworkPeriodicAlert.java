package frc.lib.networkalerts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class NetworkPeriodicAlert {
  private static final String defaultGroupName = "NetworkAlerts";
  private final Supplier<String> messageSupplier;
  private final BooleanSupplier isActive;
  private final String groupName;
  private final Alert alert;

  public NetworkPeriodicAlert(String groupName, Supplier<String> messageSupplier, AlertType alertType, BooleanSupplier isActive) {
      if (groupName == null) groupName = defaultGroupName;
      this.messageSupplier = messageSupplier;
      this.alert = new Alert(groupName, messageSupplier.get(), alertType);
      this.groupName = groupName;
      this.isActive = isActive;
  }

  protected void update() {
      alert.setText(messageSupplier.get());
      alert.set(isActive.getAsBoolean());
  }

  public boolean getIsActive() {
    return isActive.getAsBoolean();
  }

  public String getMessage() {
    return messageSupplier.get();
  }

  public AlertType getAlertType() {
    return alert.getType();
  }

  public String getGroup() {
    return groupName;
  }
}
