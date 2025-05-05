package frc.robot.subsystems.pivot.io;

import static frc.robot.subsystems.pivot.PivotConstants.Sim.*;

import java.util.Map;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import static frc.robot.subsystems.pivot.PivotConstants.ANGLE_OFFSET;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.NetworkPeriodicAlert;
import frc.robot.utils.AlertsFactory;

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
    protected Map<String, NetworkPeriodicAlert> getMotorAlerts() {
        return AlertsFactory.revMotor(
            REVLibError.kOk, () -> new Warnings(0), () -> new Faults(0), "Pivot", "Motor");
    }
}