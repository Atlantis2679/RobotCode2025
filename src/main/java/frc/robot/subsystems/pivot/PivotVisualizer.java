package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.logfields.LogFieldsTable;

public class PivotVisualizer {
    private final LogFieldsTable fieldsTable;
    private final String name;

    private final LoggedMechanism2d pivotMech = new LoggedMechanism2d(0, 0);
    private final LoggedMechanismRoot2d root = pivotMech.getRoot("root", 0, 0);
    private final LoggedMechanismLigament2d tower = root
            .append(new LoggedMechanismLigament2d("tower", 0, 90, 0, new Color8Bit(Color.kBrown)));
    private final LoggedMechanismLigament2d pivotLigament;

    public PivotVisualizer(LogFieldsTable fieldsTable, String name, Color8Bit color) {
        this.fieldsTable = fieldsTable;
        this.name = name;

        pivotLigament = tower
                .append(new LoggedMechanismLigament2d("pivot", 0.5, -90, 6, color));
    }

    public void update(double angleDegrees) {
        pivotLigament.setAngle(angleDegrees);
        fieldsTable.recordOutput(name, pivotMech);
    }
}
