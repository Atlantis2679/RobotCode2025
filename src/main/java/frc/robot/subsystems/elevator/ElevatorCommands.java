package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorCommands {
  private Elevator elevator;

  public ElevatorCommands(Elevator elevator){
    this.elevator = elevator;
  }
  public void setHeight(double desiredheight){
    elevator.setVoltage();
  }
  public double getHeight(){
    return MOTOR_HEIGHT + CARRIAGE_LENGTH * Math.cos(elevator.getAngle() * 2 * Math.PI);
  }
}
