package frc.robot.subsystems.funnel.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.NetworkAlertsManager;

import static frc.robot.RobotMap.*;
import static frc.robot.subsystems.funnel.FunnelConstants.MAX_CURRENT;

public class FunnelIOSparksMax extends FunnelIO {
    private SparkMax funnelMotor = new SparkMax(CANBUS.FUNNEL_MOTOR_ID, MotorType.kBrushless);
    private DigitalInput beamBrake = new DigitalInput(FUNNEL_BEAM_BRAKE_ID);

    private SparkMaxConfig motorConfig = new SparkMaxConfig();

    public FunnelIOSparksMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        motorConfig.smartCurrentLimit(MAX_CURRENT);
        NetworkAlertsManager.addRevLibErrorAlert("Funnel: Motor Config: ", () ->
            funnelMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
        NetworkAlertsManager.addSparkMotorAlert("Funnel: Motor: ", funnelMotor::getFaults, funnelMotor::getWarnings);
    }

    @Override
    public void setVoltage(double voltageDemand) {
        funnelMotor.setVoltage(voltageDemand);
    }

    @Override
    public void setPercentageSpeed(double percentageSpeed) {
        funnelMotor.set(percentageSpeed);
    }

    @Override
    protected boolean getIsCoralIn() {
        return !beamBrake.get();
    }

    @Override
    protected double getVoltage() {
        return funnelMotor.getAppliedOutput();
    }

    @Override
    protected double getCurrent() {
        return funnelMotor.getOutputCurrent();
    }
}