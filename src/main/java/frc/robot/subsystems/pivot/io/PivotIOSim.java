package frc.robot.subsystems.pivot.io;

import static frc.robot.subsystems.pivot.PivotConstants.Sim.*;

import static frc.robot.subsystems.pivot.PivotConstants.ANGLE_OFFSET;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import atlantis2679.lib.logfields.LogFieldsTable;

public class PivotIOSim extends PivotIO {
    private final SingleJointedArmSim pivotMotor = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            JOINT_GEAR_RATIO,
            JKG_METERS_SQUARED,
            ARM_LENGTH,
            Math.toRadians(TURNING_MIN_DEGREES),
            Math.toRadians(TURNING_MAX_DEGREES),
            true,
            ANGLE_OFFSET);

    public PivotIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected void periodicBeforeFields() {
        pivotMotor.update(0.02);
    }

    // Inputs:
    @Override
    protected double getMotorCurrent() {
        return pivotMotor.getCurrentDrawAmps();
    }

    @Override
    protected double getPivotAngleDegrees() {
        return Math.toDegrees(pivotMotor.getAngleRads());
    }

    // Outputs:
    @Override
    public void setVoltage(double voltage) {
        pivotMotor.setInputVoltage(-voltage);
    }

    @Override
    protected boolean getIsEncoderConnected() {
        return false;
    }
}