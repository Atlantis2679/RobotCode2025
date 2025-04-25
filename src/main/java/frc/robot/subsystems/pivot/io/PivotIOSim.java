package frc.robot.subsystems.pivot.io;

import static frc.robot.subsystems.pivot.PivotConstants.Sim.*;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import static frc.robot.subsystems.pivot.PivotConstants.ANGLE_OFFSET;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.GenericError;
import frc.robot.utils.GenericErrorGenerator;

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

    @Override
    protected GenericError getMotorError() {
        return GenericErrorGenerator.sparkMaxError(new Faults(0), "Pivot", "Motor");
    }

    @Override
    protected GenericError getMotorWarning() {
        return GenericErrorGenerator.sparkMaxWarning(new Warnings(0), "Pivot", "Motor");
    }

    @Override
    protected GenericError getMotorConfigError() {
        return GenericErrorGenerator.revError(REVLibError.kOk, "Pivot", "Motor Config");
    }
}