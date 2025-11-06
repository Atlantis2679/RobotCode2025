 package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.pivot.io.PivotIO;
import frc.robot.subsystems.pivot.io.PivotIOSparkMax;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

public class Pivot extends SubsystemBase implements Tunable{
private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

  private PIDController pivotPidController = new PIDController(PivotConstants.KP, PivotConstants.KI, PivotConstants.KD);

private final Debouncer encoderConnectedDebouncer = new Debouncer(PivotConstants.ENCODER_CONNECTED_DEBAUNCER_SEC);

public final RotationalSensorHelper sensorHelper;

private final PivotIO io = new PivotIOSparkMax(fieldsTable);

    private final TunableTrapezoidProfile pivotTrapezoid = new TunableTrapezoidProfile(
            new TrapezoidProfile.Constraints(PivotConstants.MAX_VELOCITY_DEG_PER_SEC, PivotConstants.MAX_ACCELERATION_DEG_PER_SEC_SQUARED));


          private TunableArmFeedforward pivotFeedforward =
            new TunableArmFeedforward(PivotConstants.KS, PivotConstants.KG, PivotConstants.KV, PivotConstants.KA);


  public Pivot(){
    fieldsTable.update();
    sensorHelper = new RotationalSensorHelper(io.angle.getAsDouble(), PivotConstants.ANGLE_OFFSET);
    sensorHelper.enableContinuousWrap(lowerBound, upperBound);

    TunablesManager.add("Pivot", (Tunable) this);

    // PeriodicAlertsGroup.defaultInstance.addErrorAlert(null, null);
  }

  public double getAngleDegrees(){
    return sensorHelper.getAngle();
  }

  public void periodic(){
    sensorHelper.update(io.angle.getAsDouble() * 360);
  }

  public void setPivotVolt(double volt){
    io.setVoltage(volt);
  }

  public void stop(){
    io.setVoltage(0);
  }

  public boolean isAtAngle(double angle){
    return angle == io.angle.getAsDouble() ? true : false;
  }

  public double calcVolt(TrapezoidProfile.State goal){
    double res = 0;
    MathUtil.clamp(res, -PivotConstants.PIVOT_MAX_VOLTAGE , PivotConstants.PIVOT_MAX_VOLTAGE);
    res += pivotTrapezoid.calculate(0.02, new State(sensorHelper.getAngle(), sensorHelper.getVelocity()), goal).velocity * 10;
    res += pivotPidController.calculate(sensorHelper.getAngle()) * 10;
    res += pivotFeedforward.calculate(Math.toRadians(sensorHelper.getAngle()), sensorHelper.getVelocity());


    return res;
  }

private double maxAngle = PivotConstants.MAX_ANGLE_DEGREES;
private double minAngle = PivotConstants.MIN_ANGLE_DEGREES;

private double upperBound = PivotConstants.UPPER_BOUND;
private double lowerBound = PivotConstants.LOWER_BOUND;

@Override
public void initTunable(TunableBuilder builder) {
  builder.addChild("Pivot PID", pivotPidController);
  builder.addChild("Pivot feedforward", pivotFeedforward);
  builder.addChild("Pivot Trapezoid profile", pivotTrapezoid);
  builder.addChild("Pivot rotational helper", sensorHelper);
  builder.addDoubleProperty("Pivot max angle", () -> maxAngle, (angle) -> maxAngle = angle);
  builder.addDoubleProperty("Pivot min angle", () -> minAngle, (angle) -> minAngle = angle);
  builder.addDoubleProperty("Pivot upper bound", () -> upperBound,
      (newUpperBound) -> {
          upperBound = newUpperBound;
          sensorHelper.enableContinuousWrap(lowerBound, newUpperBound);
      });
  builder.addDoubleProperty("Pivot lower bound", () -> lowerBound,
      (newLowerBound) -> {
          lowerBound = newLowerBound;
          sensorHelper.enableContinuousWrap(newLowerBound, upperBound);
      });

}
}
