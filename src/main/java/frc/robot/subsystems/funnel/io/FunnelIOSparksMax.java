package frc.robot.subsystems.funnel.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.CANBUS.*;
import static frc.robot.subsystems.funnel.FunnelConstants.*;

public class FunnelIOSparksMax extends FunnelIO {
    private final SparkMax funnelLeftMotor = new SparkMax(FUNNEL_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax funnelRightMotor = new SparkMax(FUNNEL_RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput beamBrake = new DigitalInput(FUNNEL_BEAM_BRAKE_ID);

    private final SparkMaxConfig motorsConfig = new SparkMaxConfig();

    public FunnelIOSparksMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        motorsConfig.smartCurrentLimit(MOTORS_CURRENT_LIMIT);
        funnelLeftMotor.configure(motorsConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        funnelRightMotor.configure(motorsConfig.inverted(true), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setVoltage(double voltageDemand) {
        funnelLeftMotor.setVoltage(voltageDemand);
        funnelRightMotor.setVoltage(voltageDemand);
    }

    @Override
    public void setPercentageSpeed(double percentageSpeed) {
        funnelLeftMotor.set(percentageSpeed);
        funnelRightMotor.set(percentageSpeed);
    }

    @Override
    protected boolean getIsCoralIn() {
        return beamBrake.get();
    }

    @Override
    protected double getLeftVoltage() {
        return funnelLeftMotor.getBusVoltage();
    }

    @Override
    protected double getRightVoltage() {
        return funnelRightMotor.getBusVoltage();
    }
}