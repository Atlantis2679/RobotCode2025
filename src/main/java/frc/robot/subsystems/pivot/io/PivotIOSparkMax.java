package frc.robot.subsystems.pivot.io;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.utils.NetworkAlertsMotors;

import static frc.robot.RobotMap.*;

import static frc.robot.subsystems.pivot.PivotConstants.*;

public class PivotIOSparkMax extends PivotIO {
    private final SparkMax pivotMotor = new SparkMax(CANBUS.PIVOT_MOTOR_ID, MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(PIVOT_ENCODER_ID);
    private final SparkMaxConfig config = new SparkMaxConfig();

    public PivotIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        config.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);
        REVLibError configError = pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder.setDutyCycleRange(0, 1);

        NetworkAlertsMotors.addRevLibErrorAlert("Pivot Motor Config", () -> configError);
        NetworkAlertsMotors.addMotorStuckAlert("Pivot Motor is Stuck!", motorCurrent, pivotMotor::getAppliedOutput);
        NetworkAlertsMotors.addSparkMotorAlert("Pivot Motor: ", pivotMotor::getFaults, pivotMotor::getWarnings);
    }

    // Inputs:
    @Override
    protected double getMotorCurrent() {
        return pivotMotor.getOutputCurrent();
    }

    @Override
    protected double getPivotAngleDegrees() {
        return encoder.get() * 360;
    }

    // Outputs:
    @Override
    public void setVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    @Override
    protected boolean getIsEncoderConnected() {
        return encoder.isConnected();
    }
}