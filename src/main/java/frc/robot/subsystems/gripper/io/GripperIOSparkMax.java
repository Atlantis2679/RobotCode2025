package frc.robot.subsystems.gripper.io;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.*;

public class GripperIOSparkMax extends GripperIO {
    private final SparkMax rightOutTakeMotor = new SparkMax(GRIPPER_RIGHT_OUTTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftOutTakeMotor = new SparkMax(GRIPPER_LEFT_OUTTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax intakeOutTakeMotor = new SparkMax(GRIPPER_INTAKE_OUTTAKE_MOTOR_ID, MotorType.kBrushless);
    
    private final RelativeEncoder rightOutTakeEncoder = rightOutTakeMotor.getEncoder();
    private final RelativeEncoder leftOutTakeEncoder = leftOutTakeMotor.getEncoder();
    private final RelativeEncoder intakeOutTakeEncoder = intakeOutTakeMotor.getEncoder();

    private final DigitalInput beamBrake = new DigitalInput(GRIPPER_BEAM_BRAKE_ID); 

    public GripperIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected boolean getIsCoralIn() {
        return beamBrake.get();
    }

    @Override
    public void setVoltageRightOutTake(double voltage) {
        rightOutTakeMotor.setVoltage(voltage);
    }

    @Override
    public void setPrecentageSpeedRightOutTake(double precentageSpeed) {
        rightOutTakeMotor.set(precentageSpeed);
    }

    @Override
    public void setVoltageLeftOutTake(double voltage) {
        leftOutTakeMotor.setVoltage(voltage);
    }

    @Override
    public void setPrecentageSpeedLeftOutTake(double precentageSpeed) {
        leftOutTakeMotor.set(precentageSpeed);
    }

    @Override
    public void setVoltageIntakeOutTake(double voltage) {
        intakeOutTakeMotor.setVoltage(voltage);
    }

    @Override
    public void setPrecentageSpeedIntakeOutTake(double precentageSpeed) {
        intakeOutTakeMotor.set(precentageSpeed);
    }

    @Override
    protected double getRightOutTakeMotorSpeedRPM() {
        return rightOutTakeEncoder.getVelocity();
    }

    @Override
    protected double getLeftOutTakeMotorSpeedRPM() {
        return leftOutTakeEncoder.getVelocity();
    }

    @Override
    protected double getIntakeOutTakeMotorSpeedRPM() {
        return intakeOutTakeEncoder.getVelocity();
    }
}
