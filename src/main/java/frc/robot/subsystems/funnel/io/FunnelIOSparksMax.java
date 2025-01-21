package frc.robot.subsystems.funnel.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.*;

public class FunnelIOSparksMax extends FunnelIO {
    private SparkMax funnelMotor = new SparkMax(FUNNEL_MOTOR_ID, MotorType.kBrushless);
    private DigitalInput beamBrake = new DigitalInput(FUNNEL_BEAM_BRAKE_ID);

    public FunnelIOSparksMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
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
        return beamBrake.get();
    }

}