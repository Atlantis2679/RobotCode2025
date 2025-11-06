package frc.robot.subsystems.pivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotCommands {
  

  private Pivot pivot;

  public PivotCommands(Pivot pivot){
    this.pivot = pivot;
  }

  public Command moveToAngle(DoubleSupplier angle){
    return pivot.run(() -> pivot.setPivotVolt(pivot.calcVolt(new TrapezoidProfile.State(angle.getAsDouble(), 0))));
  }

  public Command moveToAngle(double angle){
    return moveToAngle(() -> angle);
  }

  public Command manualController(DoubleSupplier speed){
    return moveToAngle(speed);
  }
}
