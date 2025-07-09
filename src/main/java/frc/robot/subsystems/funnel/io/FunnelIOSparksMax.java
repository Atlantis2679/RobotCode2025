package frc.robot.subsystems.funnel.io;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import frc.robot.RobotMap.CANBUS;
import frc.robot.utils.AlertsFactory;

import static frc.robot.RobotMap.*;
import static frc.robot.subsystems.funnel.FunnelConstants.MAX_CURRENT;

public class FunnelIOSparksMax extends FunnelIO {
    private SparkMax motor = new SparkMax(CANBUS.FUNNEL_MOTOR_ID, MotorType.kBrushless);
    private DigitalInput beamBrake = new DigitalInput(FUNNEL_BEAM_BRAKE_ID);

    private SparkMaxConfig motorConfig = new SparkMaxConfig();

    public FunnelIOSparksMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        motorConfig.smartCurrentLimit(MAX_CURRENT);
        motorConfig.idleMode(IdleMode.kCoast);

        REVLibError motorConfigError = motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance, "Funnel: Motor",
            () -> motorConfigError, motor::getWarnings, motor::getFaults);
    }
    
    @Override
    public void setPercentageSpeed(double percentageSpeed) {
        motor.set(percentageSpeed);
    }

    @Override
    protected boolean getIsCoralIn() {
        return !beamBrake.get();
    }

    @Override
    protected double getCurrent() {
        return motor.getOutputCurrent();
    }
}