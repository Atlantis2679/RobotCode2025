package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorCommands {
  private Elevator elevator;
  private PIDController pidController = new PIDController(0, 0, 0);


  public ElevatorCommands(Elevator elevator){
    this.elevator = elevator;
  }
  public double getHeight(){
    return MOTOR_HEIGHT + CARRIAGE_LENGTH * Math.cos(elevator.getEncoderAngle() * 2 * Math.PI);
  }

}
