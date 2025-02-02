package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.FULL_ROTATION;
import static frc.robot.subsystems.pivot.PivotConstants.INITIAL_OFFSET;
import static frc.robot.subsystems.pivot.PivotConstants.KA;
import static frc.robot.subsystems.pivot.PivotConstants.KD;
import static frc.robot.subsystems.pivot.PivotConstants.KG;
import static frc.robot.subsystems.pivot.PivotConstants.KI;
import static frc.robot.subsystems.pivot.PivotConstants.KP;
import static frc.robot.subsystems.pivot.PivotConstants.KS;
import static frc.robot.subsystems.pivot.PivotConstants.KV;
import static frc.robot.subsystems.pivot.PivotConstants.MAX_ANGLE_DIFFERENCE;
import static frc.robot.subsystems.pivot.PivotConstants.MAX_SPEED;
import static frc.robot.subsystems.pivot.PivotConstants.MIN_SPEED;
import static frc.robot.subsystems.pivot.PivotConstants.SIM_KA;
import static frc.robot.subsystems.pivot.PivotConstants.SIM_KG;
import static frc.robot.subsystems.pivot.PivotConstants.SIM_KS;
import static frc.robot.subsystems.pivot.PivotConstants.SIM_KV;
import static frc.robot.subsystems.pivot.PivotConstants.UPPER_BOUND;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.Robot;
import frc.robot.subsystems.pivot.io.PivotIO;
import frc.robot.subsystems.pivot.io.PivotIOSim;
import frc.robot.subsystems.pivot.io.PivotIOSparxmax;
import frc.robot.utils.PrimitiveRotationalSensorHelper;

public class Pivot extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final PivotIO io = Robot.isSimulation() ? new PivotIOSim(fieldsTable) : new PivotIOSparxmax(fieldsTable);
    private final PrimitiveRotationalSensorHelper pivotRotationalHelper;

    private final TrapezoidProfile pivotTrapezoid = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_SPEED, MIN_SPEED));

    private PIDController pivotPidController = new PIDController(KP, KI, KD);

    private ArmFeedforward pivotFeedforward = Robot.isSimulation() ? new ArmFeedforward(SIM_KS, SIM_KG, SIM_KV, SIM_KA)
        : new ArmFeedforward(KS, KG, KV, KA);

    public Pivot() {
        fieldsTable.update();
        pivotRotationalHelper = new PrimitiveRotationalSensorHelper(io.speed.getAsDouble(), INITIAL_OFFSET);
        pivotRotationalHelper.enableContinousWrap(UPPER_BOUND, FULL_ROTATION);
    }

    @Override
    public void periodic() {
        fieldsTable.update();
        pivotRotationalHelper.update(io.angle.getAsDouble());
    }

    public void setPivotSpeed(double speed) {
        fieldsTable.recordOutput("Desired Speed", speed);
        speed = MathUtil.clamp(speed, -1, 1);
        fieldsTable.recordOutput("Real Speed", io.speed.getAsDouble());
        io.setSpeed(speed);
    }

    public void stop() {
        setPivotSpeed(0);
    }

    public double getAngleDegrees() {
        return pivotRotationalHelper.getAngle();
    }

    public double calculateFeedForward(double desiredAngleDegrees, double desiredSpeed, boolean usePID) {
        double speed = pivotFeedforward.calculate(Math.toRadians(desiredAngleDegrees), desiredSpeed);
        if(usePID) {
            speed += pivotPidController.calculate(getAngleDegrees(), desiredAngleDegrees);
        }

        return speed;
    }

    public boolean isAtAngle(double desiredAngleDegrees) {
        return Math.abs(desiredAngleDegrees - getAngleDegrees()) < MAX_ANGLE_DIFFERENCE;
    }
}
