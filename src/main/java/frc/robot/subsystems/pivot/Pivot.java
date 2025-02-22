package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;
import static frc.robot.subsystems.swerve.SwerveContants.MAX_VOLTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableArmFeedforward;
import frc.lib.tuneables.extensions.TuneableTrapezoidProfile;
import frc.robot.Robot;
import frc.robot.subsystems.pivot.PivotConstants.Sim;
import frc.robot.subsystems.pivot.io.PivotIO;
import frc.robot.subsystems.pivot.io.PivotIOSim;
import frc.robot.subsystems.pivot.io.PivotIOSparkMax;
import frc.robot.utils.PrimitiveRotationalSensorHelper;

public class Pivot extends SubsystemBase implements Tuneable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final PivotIO io = Robot.isSimulation() ? new PivotIOSim(fieldsTable) :new PivotIOSparkMax(fieldsTable);
    
    private final PivotVisualizer pivotVisualizer = new PivotVisualizer(fieldsTable, "Real Mech2d",
            new Color8Bit(Color.kPurple));
    private final PivotVisualizer desiredPivotVisualizer = new PivotVisualizer(fieldsTable, "Desired Visualizer",
            new Color8Bit(Color.kYellow));
    
    private final PrimitiveRotationalSensorHelper pivotRotationalHelper;

    private final TuneableTrapezoidProfile pivotTrapezoid = new TuneableTrapezoidProfile(
        new TrapezoidProfile.Constraints(MAX_VELOCITY_DEG_PER_SEC, MAX_ACCELERATION));

    private PIDController pivotPidController = new PIDController(KP, KI, KD);

    private TuneableArmFeedforward pivotFeedforward = Robot.isSimulation() ? new TuneableArmFeedforward(Sim.SIM_KS, Sim.SIM_KG, Sim.SIM_KV, Sim.SIM_KA)
        : new TuneableArmFeedforward(KS, KG, KV, KA);


    private double lastDesiredVoltage = 0;

    private double maxAngle = MAX_ANGLE_DEGREES;
    private double minAngle = MIN_ANGLE_DEGREES;

    private double upperBound = UPPER_BOUND;

    public Pivot() {
        fieldsTable.update();
        pivotRotationalHelper = new PrimitiveRotationalSensorHelper(io.angle.getAsDouble(), INITIAL_OFFSET);
        pivotRotationalHelper.enableContinousWrap(UPPER_BOUND, FULL_ROTATION);
        TuneablesManager.add("Pivot", (Tuneable) this);
    }

    @Override
    public void periodic() {
        pivotRotationalHelper.update(io.angle.getAsDouble());
        pivotVisualizer.update(getAngleDegrees());
        fieldsTable.recordOutput("angle", getAngleDegrees());
        fieldsTable.recordOutput("Desired Voltage", lastDesiredVoltage);
        fieldsTable.recordOutput("rotaionHelper", getAngleDegrees());
        fieldsTable.recordOutput("feedForWord", pivotFeedforward.getArmFeedforward());
        fieldsTable.recordOutput("velocity", pivotRotationalHelper.getVelocity() * Math.PI / 180);
    }

    public void setPivotVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
        if((getAngleDegrees() > maxAngle && voltage > 0)
            || (getAngleDegrees() < minAngle && voltage < 0)) {
            voltage = 0;
        }
        lastDesiredVoltage = voltage;
        io.setVoltage(voltage);
    }

    public void stop() {
        lastDesiredVoltage = 0;
        io.setVoltage(0);
    }

    public double getAngleDegrees() {
        return pivotRotationalHelper.getAngle();
    }
    
    public double getVelocity() {
        return pivotRotationalHelper.getVelocity();
    }

    public double calculateFeedForward(double desiredAngleDegrees, double desiredSpeed, boolean usePID) {
        desiredPivotVisualizer.update(getAngleDegrees());
        double speed = pivotFeedforward.calculate(Math.toRadians(desiredAngleDegrees), desiredSpeed);
        if(usePID) {
            speed += pivotPidController.calculate(getAngleDegrees(), desiredAngleDegrees);
        }
        return speed;
    }

    public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
            TrapezoidProfile.State goalState) {
        return pivotTrapezoid.calculate(time, initialState, goalState);
    }

    public boolean isAtAngle(double desiredAngleDegrees) {
        fieldsTable.recordOutput("desored angle", desiredAngleDegrees);
        return Math.abs(desiredAngleDegrees - getAngleDegrees()) < MESURED_ANGLE_TOLERENCE_DEGREES;
    }

    public void resetPID() {
        pivotPidController.reset();
    }

    public void initTuneable(TuneableBuilder builder) {
        builder.addChild("Pivot PID", pivotPidController);
        builder.addChild("Pivot feedforward", pivotFeedforward);
        builder.addChild("Pivot Trapezoid profile", pivotTrapezoid);
        builder.addChild("Pivot angle degrees", pivotRotationalHelper);
        builder.addDoubleProperty("Pivot max angle", () -> maxAngle, (angle) -> maxAngle = angle);
        builder.addDoubleProperty("Pivot min angle", () -> minAngle, (angle) -> minAngle = angle);
        builder.addDoubleProperty("Pivot upper bound", () -> upperBound,
            (newUpperBound) -> {
                upperBound = newUpperBound;
                pivotRotationalHelper.enableContinousWrap(newUpperBound, FULL_ROTATION);
            });
    }
}
