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
            new TrapezoidProfile.Constraints(MAX_VELOCITY_DEG_PER_SEC, MAX_ACCELERATION_DEG_PER_SEC_SQUARED));

  private PIDController elevatorPidController = new PIDController(KP, KI, KD);

  private TunableArmFeedforward elevatorFeedforward = Robot.isReal()
            ? new TunableArmFeedforward(KS, KG, KV)
            : new TunableArmFeedforward(Sim.SIM_KS, Sim.SIM_KG, Sim.SIM_KV, Sim.SIM_KA);

    private double maxAngle = MAX_ANGLE_DEGREES;
    private double minAngle = MIN_ANGLE_DEGREES;

    private final Debouncer encoderConnectedDebouncer = new Debouncer(ENCODER_CONNECTED_DEBAUNCER_SEC);

  public Elevator(){
    fieldsTable.update();
    elevatorRotationalSensorHelper = new RotationalSensorHelper(io.encoderAngle.getAsDouble(), ANGLE_OFFSET);

    TunablesManager.add("Elevator", (Tunable) this);

    PeriodicAlertsGroup.defaultInstance.addErrorAlert(() -> "Elevator: Encoder Disconnected!", () -> !getEncoderConnectedDebouncer());
  }

  @Override
  public void periodic() {
    fieldsTable.recordOutput("Encoder angle", io.getEncoderAngle());
    fieldsTable.recordOutput("Right motor current", io.getRightMotorCurrent());
    fieldsTable.recordOutput("Left motor current", io.getLeftMotorCurrent());
  }
  public void setVoltage(double voltage) {
    if((getAngle() > MAX_ANGLE_DEGREES && voltage > 0)
          || (getAngle() < MIN_ANGLE_DEGREES && voltage < 0)) {
      voltage = 0.0;
    }
    voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
    fieldsTable.recordOutput("voltage", voltage);
    io.setVoltage(voltage);
  }
  public void stop(){
    io.setVoltage(0);
  }
  public double getAngle(){
    return io.getEncoderAngle();
  }
  public double getVelocity() {
    return elevatorRotationalSensorHelper.getVelocity();
  }
  public boolean getEncoderConnectedDebouncer() {
    return encoderConnectedDebouncer.calculate(io.isEncoderConnected.getAsBoolean());
  }
  public double calculateFeedForward(double desiredAngleDegrees, double desiredSpeed, boolean usePID) {
    fieldsTable.recordOutput("desired angle", desiredAngleDegrees);
    fieldsTable.recordOutput("desired speed", desiredSpeed);
    double speed = elevatorFeedforward.calculate(Math.toRadians(desiredAngleDegrees), desiredSpeed);
    if (usePID && !isAtAngle(desiredAngleDegrees)) {
        speed += elevatorPidController.calculate(getAngle(), desiredAngleDegrees);
    }
    return speed;
  }
  public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
            TrapezoidProfile.State goalState) {
        return elevatorTrapezoid.calculate(time, initialState, goalState);
  }
  public boolean isAtAngle(double desiredAngleDegrees) {
    return Math.abs(desiredAngleDegrees - getAngle()) <= ANGLE_TOLERENCE_DEGREES;
  }
  public void resetPID() {
    elevatorPidController.reset();
  }
}
