package frc.robot.subsystems.funnel.funnelIO;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class FunnelIOSparkMax extends FunnelIO{
    
    private SparkMax motor = new SparkMax(RobotMap.CANBUS.FUNNEL_MOTOR_ID, MotorType.kBrushless);
    private SparkMaxConfig motorConfig = new SparkMaxConfig();
    private DigitalInput beamBreak = new DigitalInput(RobotMap.FUNNEL_BEAM_BRAKE_ID);

    public FunnelIOSparkMax(LogFieldsTable logFieldsTable){
        super(logFieldsTable);
        motorConfig.idleMode(IdleMode.kCoast);
        motorConfig.smartCurrentLimit(FunnelConstants.MAX_CURRENT);

        REVLibError motorConfigError = motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance, "Funnel Config", () -> motorConfigError, motor::getWarnings, motor::getFaults);
    }

    //Output Methods:
    @Override
    protected boolean getBeamBreak(){
        return !beamBreak.get();
    }

    @Override
    protected double getMotorCurrent(){
        return motor.getOutputCurrent();
    }

    //Input methods:
    @Override
    public void setPrecentageSpeed(double speed){
        motor.set(speed);
    }
}
