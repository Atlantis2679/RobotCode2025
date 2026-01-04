package frc.robot.subsystems.elevator.io;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class ElevatorIOSim extends ElevatorIO {
  public final ElevatorSim elevatorSim = new ElevatorSim(
      DCMotor.getNEO(1),
      JOINT_GEAR_RATIO,
      CARRAGE_MASS_KG,
      DRUM_RADIUS_METERS,
      MIN_HEIGHT_METERS,
      MAX_HEIGHT_METERS,
      true,
      STARTING_HEIGHT_METERS,
      measurementStdDevs);

  public ElevatorIOSim(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  // Outputs:

  @Override
  public double getHeightMeters() {
    return elevatorSim.getPositionMeters();
  }

  @Override
  public double getRightMotorCurrent() {
    return elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public double getLeftMotorCurrent() {
    return elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public boolean getIsEncoderConnected() {
    return true;
  }
  // Inputs:

  @Override
  public void setVoltage(double voltage) {
    elevatorSim.setInputVoltage(voltage);
  }
}
