package frc.robot.subsystems.hood.io;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import frc.robot.utils.AlertsFactory;

import static frc.robot.RobotMap.*;


import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodIOSparkMax extends HoodIO {
    private final SparkMax hoodMotor = new SparkMax(CANBUS.HOOD_MOTOR_ID, MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(HOOD_ENCODER_ID);
    private final SparkMaxConfig config = new SparkMaxConfig();

    public HoodIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        config.smartCurrentLimit(HOOD_CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);
        REVLibError configError = hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        encoder.setDutyCycleRange(0, 1);

        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance, "Pivot config", () -> configError, hoodMotor::getWarnings, hoodMotor::getFaults);
    }

    // Inputs:
    @Override
    protected double getMotorCurrent() {
        return hoodMotor.getOutputCurrent();
    }

    @Override
    protected double getHoodAngleDegrees() {
        return encoder.get() * 360;
    }

    // Outputs:
    @Override
    public void setVoltage(double voltage) {
        hoodMotor.setVoltage(voltage);
    }

    @Override
    protected boolean getIsEncoderConnected() {
        return encoder.isConnected();
    }
}