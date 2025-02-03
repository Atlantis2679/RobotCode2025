package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesManager;
import frc.robot.Robot;
import frc.robot.subsystems.pivot.io.PivotIO;
import frc.robot.subsystems.pivot.io.PivotIOSim;
import frc.robot.subsystems.pivot.io.PivotIOSparxmax;
import frc.robot.utils.PrimitiveRotationalSensorHelper;

public class Pivot extends SubsystemBase implements Tuneable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final PivotIO io = Robot.isSimulation() ? new PivotIOSim(fieldsTable) : new PivotIOSparxmax(fieldsTable);
    
    private final PivotVisualizer pivotVisualizer = new PivotVisualizer(fieldsTable, "Real Mech2d",
            new Color8Bit(Color.kPurple));
    private final PivotVisualizer desiredPivotVisualizer = new PivotVisualizer(fieldsTable, "Desired Visualizer",
            new Color8Bit(Color.kYellow));

    private final PrimitiveRotationalSensorHelper pivotRotationalHelper;

    private final TrapezoidProfile pivotTrapezoid = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

    private PIDController pivotPidController = new PIDController(KP, KI, KD);

    private ArmFeedforward pivotFeedforward = Robot.isSimulation() ? new ArmFeedforward(Sim.SIM_KS, Sim.SIM_KG, Sim.SIM_KV, Sim.SIM_KA)
        : new ArmFeedforward(KS, KG, KV, KA);

    private double lastDesiredVoltage = 0;

    public Pivot() {
        pivotRotationalHelper = new PrimitiveRotationalSensorHelper(io.angle.getAsDouble(), INITIAL_OFFSET);
        pivotRotationalHelper.enableContinousWrap(UPPER_BOUND, FULL_ROTATION);
        TuneablesManager.add("Pivot", (Tuneable) this);
    }

    @Override
    public void periodic() {
        pivotRotationalHelper.update(io.angle.getAsDouble());
        pivotVisualizer.update(getAngleDegrees());
        fieldsTable.recordOutput("Desired Voltage", lastDesiredVoltage);
    }

    public void setPivotVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, 0, PIVOT_CURRENT_LIMIT);
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
        return Math.abs(desiredAngleDegrees - getAngleDegrees()) < MESURED_ANGLE_TOLERENCE_DEGREES;
    }

    public void initTuneable(TuneableBuilder builder) {
        builder.addChild("Pivot Subsystem", (Sendable) this);
    }

    public void resetPID() {
        pivotPidController.reset();
    }
}
