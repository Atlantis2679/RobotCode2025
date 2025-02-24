package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.logfields.LogFieldsTable;

public class PivotVisualizer {
    private final LogFieldsTable fieldsTable;
    private final String name;

    private final Mechanism2d pivotMech = new Mechanism2d(0, 0);
    private final MechanismRoot2d root = pivotMech.getRoot("root", 0, 0);
    private final MechanismLigament2d tower = root.append(new MechanismLigament2d("tower", 0, 90, 0, new Color8Bit(Color.kBrown)));
    private final MechanismLigament2d pivotLigament;

    public PivotVisualizer(LogFieldsTable fieldsTable, String name, Color8Bit color) {
        this.fieldsTable = fieldsTable;
        this.name = name;

        pivotLigament = tower
                .append(new MechanismLigament2d("pivot", 0.5, -90, 6, color));
    }

    public void update(double angleDegrees) {
        pivotLigament.setAngle(angleDegrees);
        fieldsTable.recordOutput(name, pivotMech.toString());
    }
}