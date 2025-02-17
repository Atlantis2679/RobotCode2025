package frc.robot.subsystems.gripper.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.*;
import static frc.robot.subsystems.gripper.GripperConstants.*;

public class GripperIOSparkMax extends GripperIO {
    private final SparkMax rightMotor = new SparkMax(CANBUS.GRIPPER_RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(CANBUS.GRIPPER_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

    private final DigitalInput beamBrake = new DigitalInput(GRIPPER_BEAM_BRAKE_ID);

    public GripperIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        leftMotorConfig.smartCurrentLimit(LEFT_MOTOR_CURRENT_LIMIT);
        rightMotorConfig.smartCurrentLimit(RIGHT_MOTOR_CURRENT_LIMIT);
        rightMotorConfig.inverted(true);
        rightMotor.configure(leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        leftMotor.configure(rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Inputs:

    // Outputs:

    @Override
    protected int getRightOuttakeMotorStatusValue() {
        return rightMotor.getLastError().value;
    }

    @Override
    protected int getLeftOuttakeMotorStatusValue() {
        return leftMotor.getLastError().value;
    }

    @Override
    protected boolean getIsCoralIn() {
        return !beamBrake.get();
    }
    
    @Override
    protected double getLeftMotorVoltage() {
        return leftMotor.getAppliedOutput();
    }

    @Override
    protected double getRightMotorVoltage() {
        return rightMotor.getAppliedOutput();
    }

    // Outputs:

    @Override
    public void setRightMotorVoltage(double voltage) {
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void setLeftMotorVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }
}
