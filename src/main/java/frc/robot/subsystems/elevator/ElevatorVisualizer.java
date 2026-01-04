package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color8Bit;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class ElevatorVisualizer {
  private final LogFieldsTable fieldsTable;
  private String name;

  private final LoggedMechanism2d elevatorMech = new LoggedMechanism2d(1.5, 1.5);
  private final LoggedMechanismRoot2d elevatorRoot = elevatorMech.getRoot("root", 0.75, 0);
  private final LoggedMechanismLigament2d elevatorTower;

  ElevatorVisualizer(LogFieldsTable fieldsTable, String name, Color8Bit color) {
    this.fieldsTable = fieldsTable;
    this.name = name;

    elevatorTower = elevatorRoot.append(new LoggedMechanismLigament2d("tower", 0.5, 90, 2, color));
  }

  public void update(double height) {
    elevatorTower.setLength(height + 0.3);
    fieldsTable.recordOutput("vizualizer height", height);
    fieldsTable.recordOutput(name, elevatorMech);
  }
}
