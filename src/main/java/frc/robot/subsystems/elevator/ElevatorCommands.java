package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.valueholders.ValueHolder;

public class ElevatorCommands {
  private Elevator elevator;

  public ElevatorCommands(Elevator elevator) {
    this.elevator = elevator;
  }

  public Command moveToAngle(DoubleSupplier desiredAngleDeg) {
    ValueHolder<TrapezoidProfile.State> referenceState = new ValueHolder<TrapezoidProfile.State>(null);
    return elevator.runOnce(() -> {
      elevator.resetPID();
      referenceState.set(new TrapezoidProfile.State(elevator.getAngle(), elevator.getVelocity()));
    }).andThen(elevator.run(() -> {
      // if (referenceState.get().position == desiredAngleDeg.getAsDouble()
      // && Math.abs(elevator.getAngleDegrees() - desiredAngleDeg.getAsDouble()) > 40)
      // referenceState.set(new TrapezoidProfile.State(elevator.getAngleDegrees(),
      // elevator.getVelocity()));

      referenceState.set(elevator.calculateTrapezoidProfile(
          0.02,
          referenceState.get(),
          new TrapezoidProfile.State(desiredAngleDeg.getAsDouble(), 0)));

      double voltage = elevator.calculateFeedForward(
          referenceState.get().position,
          referenceState.get().velocity,
          true);

      elevator.setVoltage(voltage);
    })).withName("elevatorMoveToAngle");
  }

  public Command setHeight(double desiredHeight) {
    double ratio = (desiredHeight - MOTOR_HEIGHT) / CARRIAGE_LENGTH;

    // Safety clamp (acos only works for [-1, 1])
    ratio = Math.max(-1.0, Math.min(1.0, ratio));

    double angle = Math.acos(ratio) / (2 * Math.PI);

    return moveToAngle(() -> angle);
  }

  public Command manualController(DoubleSupplier elevatorSpeed) {
    return elevator.run(() -> {
      double demandSpeed = elevatorSpeed.getAsDouble();

      double feedForward = elevator.calculateFeedForward(elevator.getAngle(), 0, false);

      elevator.setVoltage(feedForward + demandSpeed * MAX_VOLTAGE);
    }).withName("elevatorManualController");
  }
}
