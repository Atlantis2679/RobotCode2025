package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.io.ElevatorIO;
import frc.robot.subsystems.elevator.io.ElevatorIOSim;
import frc.robot.subsystems.elevator.io.ElevatorIOSparkMax;
import frc.robot.subsystems.pivot.PivotConstants.Sim;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
  private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

  private final ElevatorIO io = Robot.isReal() ? new ElevatorIOSparkMax(fieldsTable) : new ElevatorIOSim(fieldsTable);

  private final RotationalSensorHelper elevatorRotationalSensorHelper;

  private final TunableTrapezoidProfile elevatorTrapezoid = new TunableTrapezoidProfile(
      new TrapezoidProfile.Constraints(MAX_VELOCITY_METERS_PER_SEC, MAX_ACCELERATION_METER_PER_SEC_SQUARED));

  private PIDController elevatorPidController = new PIDController(KP, KI, KD);

  private TunableArmFeedforward elevatorFeedforward = Robot.isReal()
      ? new TunableArmFeedforward(KS, KG, KV)
      : new TunableArmFeedforward(Sim.SIM_KS, Sim.SIM_KG, Sim.SIM_KV, Sim.SIM_KA);

  private final Debouncer encoderConnectedDebouncer = new Debouncer(ENCODER_CONNECTED_DEBAUNCER_SEC);

  public Elevator() {
    fieldsTable.update();
    elevatorRotationalSensorHelper = new RotationalSensorHelper(io.encoderAngle.getAsDouble(), ANGLE_OFFSET);

    TunablesManager.add("Elevator", (Tunable) this);

    PeriodicAlertsGroup.defaultInstance.addErrorAlert(() -> "Elevator: Encoder Disconnected!",
        () -> !getEncoderConnectedDebouncer());
  }

  @Override
  public void periodic() {
    fieldsTable.recordOutput("Encoder angle", io.getEncoderAngle());
    fieldsTable.recordOutput("Right motor current", io.getRightMotorCurrent());
    fieldsTable.recordOutput("Left motor current", io.getLeftMotorCurrent());
  }

  public void setVoltage(double voltage) {
    if ((getAngle() > MAX_ANGLE_DEGREES && voltage > 0)
        || (getAngle() < MIN_ANGLE_DEGREES && voltage < 0)) {
      voltage = 0.0;
    }
    voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
    fieldsTable.recordOutput("voltage", voltage);
    io.setVoltage(voltage);
  }

  public void stop() {
    io.setVoltage(0);
  }

  private double getAngle() {
    return io.getEncoderAngle();
  }

  private double angleToHeight(double angle) {
    return MOTOR_HEIGHT
        + CARRIAGE_LENGTH
            * Math.cos(angle / 360 * 2 * Math.PI);
  }

  public double getHeight() {
    return angleToHeight(getAngle());
  }

  public double getAngularVelocity() {
    return elevatorRotationalSensorHelper.getVelocity();
  }
  public double getHeightVelocity() {
    return -CARRIAGE_LENGTH * 2 * Math.PI * Math.sin(getAngle() / 360 * 2 * Math.PI) * getAngularVelocity();
  }

  public boolean getEncoderConnectedDebouncer() {
    return encoderConnectedDebouncer.calculate(io.isEncoderConnected.getAsBoolean());
  }

  public double calculateFeedForward(double desiredHeight, double desiredSpeed, boolean usePID) {
    fieldsTable.recordOutput("desired  height", desiredHeight);
    fieldsTable.recordOutput("desired speed", desiredSpeed);
    double speed = elevatorFeedforward.calculate(desiredHeight, desiredSpeed);
    if (usePID && !isAtHeight(desiredHeight)) {
      speed += elevatorPidController.calculate(getHeight(), desiredHeight);
    }
    return speed;
  }

  public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
      TrapezoidProfile.State goalState) {
    return elevatorTrapezoid.calculate(time, initialState, goalState);
  }

  public boolean isAtHeight(double desiredHeight) {
    return Math.abs(desiredHeight - getHeight()) <= HEIGHT_TOLERENCE;
  }

  public void resetPID() {
    elevatorPidController.reset();
  }
}
