
package frc.robot.subsystems.hood;

import static frc.robot.subsystems.swerve.SwerveContants.MAX_VOLTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.hood.HoodConstants.Sim;
import frc.robot.subsystems.hood.io.HoodIO;
import frc.robot.subsystems.hood.io.HoodIOSim;
import frc.robot.subsystems.hood.io.HoodIOSparkMax;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

import static frc.robot.subsystems.hood.HoodConstants.*;

public class Hood extends SubsystemBase implements Tunable{
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final HoodIO io = Robot.isSimulation() ? new HoodIOSim(fieldsTable) : new HoodIOSparkMax(fieldsTable);

    private final RotationalSensorHelper hoodRotationalHelper;

        private final HoodVisualizer realVisualizer = new HoodVisualizer(fieldsTable, " hood Real Visualizer",
            new Color8Bit(Color.kPurple));
    private final HoodVisualizer desiredHoodVisualizer = new HoodVisualizer(fieldsTable, "hood Desired Visualizer",
            new Color8Bit(Color.kYellow));

    private TunableArmFeedforward hoodFeedforward = Robot.isSimulation()
            ? new TunableArmFeedforward(Sim.SIM_KS, Sim.SIM_KG, Sim.SIM_KV, Sim.SIM_KA)
            : new TunableArmFeedforward(KS, KG, KV, KA);

    private final TunableTrapezoidProfile hoodTrapezoid = new TunableTrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_VELOCITY_DEG_PER_SEC, MAX_ACCELERATION_DEG_PER_SEC_SQUARED));

    private PIDController hoodPidController = new PIDController(KP, KI, KD);

    private double maxAngle = MAX_ANGLE_DEGREES;
    private double minAngle = MIN_ANGLE_DEGREES;

    private double upperBound = UPPER_BOUND;
    private double lowerBound = LOWER_BOUND;

    private final Debouncer encoderConnectedDebouncer = new Debouncer(ENCODER_CONNECTED_DEBAUNCER_SEC);

    public Hood() {
        fieldsTable.update();
        hoodRotationalHelper = new RotationalSensorHelper(io.angle.getAsDouble(), ANGLE_OFFSET);
        hoodRotationalHelper.enableContinuousWrap(lowerBound, upperBound);

        TunablesManager.add("Hood", (Tunable) this);

        PeriodicAlertsGroup.defaultInstance.addErrorAlert(() -> "Hood: Encoder Disconnected!", () -> !getEncoderConnectedDebouncer());
    }

    @Override
    public void periodic() {
        hoodRotationalHelper.update(io.angle.getAsDouble());
        realVisualizer.update(getAngleDegrees());

        fieldsTable.recordOutput("angle", getAngleDegrees());
        fieldsTable.recordOutput("velocity", hoodRotationalHelper.getVelocity());
        fieldsTable.recordOutput("is encoder connected", getEncoderConnectedDebouncer());
        
    }

    public void setHoodVoltage(double voltage) {
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

    public double getVelocity() {
        return hoodRotationalHelper.getVelocity();
    }

    public double getAngleDegrees() {
        return hoodRotationalHelper.getAngle();
    }

    public boolean getEncoderConnectedDebouncer() {
        return encoderConnectedDebouncer.calculate(io.isEncoderConnected.getAsBoolean());
    }

    public double calculateFeedForward(double desiredAngleDegrees, double desiredSpeed, boolean usePID) {
        fieldsTable.recordOutput("desired angle", desiredAngleDegrees);
        fieldsTable.recordOutput("desired speed", desiredSpeed);
        desiredHoodVisualizer.update(desiredAngleDegrees);
        double speed = hoodFeedforward.calculate(Math.toRadians(desiredAngleDegrees), desiredSpeed);
        if (usePID && !isAtAngle(desiredAngleDegrees)) {
            speed += hoodPidController.calculate(getAngleDegrees(), desiredAngleDegrees);
        }
        return speed;
    }

    public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
    TrapezoidProfile.State goalState) {
    return hoodTrapezoid.calculate(time, initialState, goalState);
    }

    public boolean isAtAngle(double desiredAngleDegrees) {
        return Math.abs(desiredAngleDegrees - getAngleDegrees()) < ANGLE_TOLERENCE_DEGREES;
    }

    public void resetPID() {
        hoodPidController.reset();
    }
    
        public void initTunable(TunableBuilder builder) {
        builder.addChild("Hood PID", hoodPidController);
        builder.addChild("Hood feedforward", hoodFeedforward);
        builder.addChild("Hood Trapezoid profile", hoodTrapezoid);
        builder.addChild("Hood rotational helper", hoodRotationalHelper);
        builder.addDoubleProperty("Hood max angle", () -> maxAngle, (angle) -> maxAngle = angle);
        builder.addDoubleProperty("Hood min angle", () -> minAngle, (angle) -> minAngle = angle);
        builder.addDoubleProperty("Hood upper bound", () -> upperBound,
            (newUpperBound) -> {
                upperBound = newUpperBound;
                hoodRotationalHelper.enableContinuousWrap(lowerBound, newUpperBound);
            });
        builder.addDoubleProperty("Pivot lower bound", () -> lowerBound,
            (newLowerBound) -> {
                lowerBound = newLowerBound;
                hoodRotationalHelper.enableContinuousWrap(newLowerBound, upperBound);
            });}
}
