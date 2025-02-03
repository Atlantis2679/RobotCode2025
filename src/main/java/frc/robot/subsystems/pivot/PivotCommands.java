package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotCommands {
    private final Pivot pivot;

    public PivotCommands(Pivot pivot) {
        this.pivot = pivot;
    }

    public Command manualController() {
        return pivot.run(() -> {
            
        });
    }

}
