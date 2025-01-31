package frc.robot.subsystems.funnel.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.CANBUS.*;

public class FunnelIOSparksMax extends FunnelIO {
    private SparkMax funnelLeftMotor = new SparkMax(FUNNEL_LEFT_MOTOR_ID, MotorType.kBrushless);
    private SparkMax funnelRightMotor = new SparkMax(FUNNEL_RIGHT_MOTOR_ID, MotorType.kBrushless);
    private DigitalInput beamBrake = new DigitalInput(FUNNEL_BEAM_BRAKE_ID);

    public FunnelIOSparksMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
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

}