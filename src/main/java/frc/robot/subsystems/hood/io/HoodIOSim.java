package frc.robot.subsystems.hood.io;

import static frc.robot.subsystems.hood.HoodConstants.Sim.*;

import static frc.robot.subsystems.hood.HoodConstants.ANGLE_OFFSET;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class HoodIOSim extends HoodIO {
    private final SingleJointedArmSim hoodMotor = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            JOINT_GEAR_RATIO,
            JKG_METERS_SQUARED,
            ARM_LENGTH_M,
            Math.toRadians(TURNING_MIN_DEGREES),
            Math.toRadians(TURNING_MAX_DEGREES),
            true,
            ANGLE_OFFSET);

    public HoodIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected void periodicBeforeFields() {
        hoodMotor.update(0.02);
    }

    // Inputs:
    @Override
    protected double getMotorCurrent() {
        return hoodMotor.getCurrentDrawAmps();
    }

    @Override
    protected double getHoodAngleDegrees() {
        return Math.toDegrees(hoodMotor.getAngleRads());
    }

    // Outputs:
    @Override
    public void setVoltage(double voltage) {
        hoodMotor.setInputVoltage(-voltage);
    }

    @Override
    protected boolean getIsEncoderConnected() {
        return false;
    }
}