package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;
import static frc.robot.subsystems.swerve.SwerveContants.MAX_VOLTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import frc.robot.Robot;
import frc.robot.subsystems.pivot.PivotConstants.Sim;
import frc.robot.subsystems.pivot.io.PivotIO;
import frc.robot.subsystems.pivot.io.PivotIOSim;
import frc.robot.subsystems.pivot.io.PivotIOSparkMax;

public class Pivot extends SubsystemBase implements Tunable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final PivotIO io = Robot.isSimulation() ? new PivotIOSim(fieldsTable) : new PivotIOSparkMax(fieldsTable);

    private final PivotVisualizer realVisualizer = new PivotVisualizer(fieldsTable, "Real Visualizer",
            new Color8Bit(Color.kPurple));
    private final PivotVisualizer desiredPivotVisualizer = new PivotVisualizer(fieldsTable, "Desired Visualizer",
            new Color8Bit(Color.kYellow));

    private final RotationalSensorHelper pivotRotationalHelper;

    private final TunableTrapezoidProfile pivotTrapezoid = new TunableTrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_VELOCITY_DEG_PER_SEC, MAX_ACCELERATION));

    private PIDController pivotPidController = new PIDController(KP, KI, KD);

    private TunableArmFeedforward pivotFeedforward = Robot.isSimulation()
            ? new TunableArmFeedforward(Sim.SIM_KS, Sim.SIM_KG, Sim.SIM_KV, Sim.SIM_KA)
            : new TunableArmFeedforward(KS, KG, KV, KA);

    private double maxAngle = MAX_ANGLE_DEGREES;
    private double minAngle = MIN_ANGLE_DEGREES;

    private double upperBound = UPPER_BOUND;
    private double lowerBound = LOWER_BOUND;
    
    private final Debouncer encoderConnectedDebouncer = new Debouncer(ENCODER_CONNECTED_DEBAUNCER_SEC);

    public Pivot() {
        fieldsTable.update();
        pivotRotationalHelper = new RotationalSensorHelper(io.angle.getAsDouble(), ANGLE_OFFSET);
        pivotRotationalHelper.enableContinuousWrap(lowerBound, upperBound);

        TunablesManager.add("Pivot", (Tunable) this);

        PeriodicAlertsGroup.defaultInstance.addErrorAlert(() -> "Pivot: Encoder Disconnected!", () -> !getEncoderConnectedDebouncer());
    }

    @Override
    public void periodic() {
        pivotRotationalHelper.update(io.angle.getAsDouble());
        realVisualizer.update(getAngleDegrees());

        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    
        fieldsTable.recordOutput("defualt command", getDefaultCommand() != null ? getDefaultCommand().getName() : "None");

        fieldsTable.recordOutput("angle", getAngleDegrees());
        fieldsTable.recordOutput("velocity", pivotRotationalHelper.getVelocity());
        fieldsTable.recordOutput("is encoder connected", getEncoderConnectedDebouncer());
    }

    public void setPivotVoltage(double voltage) {
        if((getAngleDegrees() > MAX_ANGLE_DEGREES && voltage > 0)
            || (getAngleDegrees() < MIN_ANGLE_DEGREES && voltage < 0)) {
            voltage = 0.0;
        }
        voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
        fieldsTable.recordOutput("voltage", voltage);
        io.setVoltage(voltage);
    }

    public void stop() {
        fieldsTable.recordOutput("voltage", 0.0);
        io.setVoltage(0);
    }

    public double getAngleDegrees() {
        return pivotRotationalHelper.getAngle();
    }

    public double getVelocity() {
        return pivotRotationalHelper.getVelocity();
    }

    public boolean getEncoderConnectedDebouncer() {
        return encoderConnectedDebouncer.calculate(io.isEncoderConnected.getAsBoolean());
    }

    public double calculateFeedForward(double desiredAngleDegrees, double desiredSpeed, boolean usePID) {
        fieldsTable.recordOutput("desired angle", desiredAngleDegrees);
        fieldsTable.recordOutput("desired speed", desiredSpeed);
        desiredPivotVisualizer.update(desiredAngleDegrees);
        double speed = pivotFeedforward.calculate(Math.toRadians(desiredAngleDegrees), desiredSpeed);
        if (usePID && !isAtAngle(desiredAngleDegrees)) {
            speed += pivotPidController.calculate(getAngleDegrees(), desiredAngleDegrees);
        }
        return speed;
    }

    public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
            TrapezoidProfile.State goalState) {
        return pivotTrapezoid.calculate(time, initialState, goalState);
    }

    public boolean isAtAngle(double desiredAngleDegrees) {
        return Math.abs(desiredAngleDegrees - getAngleDegrees()) < ANGLE_TOLERENCE_DEGREES;
    }

    public void resetPID() {
        pivotPidController.reset();
    }

    public void initTunable(TunableBuilder builder) {
        builder.addChild("Pivot PID", pivotPidController);
        builder.addChild("Pivot feedforward", pivotFeedforward);
        builder.addChild("Pivot Trapezoid profile", pivotTrapezoid);
        builder.addChild("Pivot rotational helper", pivotRotationalHelper);
        builder.addDoubleProperty("Pivot max angle", () -> maxAngle, (angle) -> maxAngle = angle);
        builder.addDoubleProperty("Pivot min angle", () -> minAngle, (angle) -> minAngle = angle);
        builder.addDoubleProperty("Pivot upper bound", () -> upperBound,
            (newUpperBound) -> {
                upperBound = newUpperBound;
                pivotRotationalHelper.enableContinuousWrap(lowerBound, newUpperBound);
            });
        builder.addDoubleProperty("Pivot lower bound", () -> lowerBound,
            (newLowerBound) -> {
                lowerBound = newLowerBound;
                pivotRotationalHelper.enableContinuousWrap(newLowerBound, upperBound);
            });
    }
}