package frc.robot.subsystems.funnel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelCommands {
  private Funnel funnle;

  public FunnelCommands(Funnel funnle){
    this.funnle = funnle;
  }

  public Command inPut(double volt){
    return funnle.run(() -> funnle.setMotorVolt(volt));
  }

  public Command manualConntroller(DoubleSupplier volt){
    return funnle.run(() -> funnle.setMotorVolt(volt.getAsDouble()));
  }

}
