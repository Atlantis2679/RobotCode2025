package frc.robot.subsystems.funnel.io;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.NetworkPeriodicAlert;
import frc.robot.RobotMap.CANBUS;
import frc.robot.utils.AlertsFactory;

import static frc.robot.RobotMap.*;
import static frc.robot.subsystems.funnel.FunnelConstants.MAX_CURRENT;

import java.util.Map;

public class FunnelIOSparksMax extends FunnelIO {
    private final SparkMax motor = new SparkMax(CANBUS.FUNNEL_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput beamBrake = new DigitalInput(FUNNEL_BEAM_BRAKE_ID);

    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private REVLibError motorConfigError = REVLibError.kOk;

    public FunnelIOSparksMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        motorConfig.smartCurrentLimit(MAX_CURRENT);
        motorConfig.idleMode(IdleMode.kCoast);

        motorConfigError = motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    protected Map<String, NetworkPeriodicAlert> getMotorAlerts() {
        return AlertsFactory.revMotor(
            () -> motorConfigError, () -> motor.getWarnings(), () -> motor.getFaults(), "Funnel", "Motor");
    }
}