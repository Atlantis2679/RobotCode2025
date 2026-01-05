package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class HoodVisualizer {
    private final LogFieldsTable fieldsTable;
    private final String name;

    private final LoggedMechanism2d hoodMech = new LoggedMechanism2d(1.5, 1.5);
    private final LoggedMechanismRoot2d root = hoodMech.getRoot("root", 0.75, 0);
    private final LoggedMechanismLigament2d tower = root
            .append(new LoggedMechanismLigament2d("tower", 0.75, 90, 2, new Color8Bit(Color.kBrown)));
    private final LoggedMechanismLigament2d hoodLigament;

    public HoodVisualizer(LogFieldsTable fieldsTable, String name, Color8Bit color) {
        this.fieldsTable = fieldsTable;
        this.name = name;

        hoodLigament = tower
                .append(new LoggedMechanismLigament2d("hood", 0.5, -90, 6, color));
    }

    public void update(double angleDegrees) {
        hoodLigament.setAngle(angleDegrees - 90);
        fieldsTable.recordOutput(name, hoodMech);
    }
}