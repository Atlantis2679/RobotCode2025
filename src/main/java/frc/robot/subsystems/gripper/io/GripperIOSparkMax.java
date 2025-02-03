package frc.robot.subsystems.gripper.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.*;
import static frc.robot.subsystems.gripper.GripperConstants.MOTORS_CURRENT_LIMIT;

public class GripperIOSparkMax extends GripperIO {
    private final SparkMax rightMotor = new SparkMax(CANBUS.GRIPPER_RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(CANBUS.GRIPPER_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig motorsConfig = new SparkMaxConfig();

    private final DigitalInput beamBrake = new DigitalInput(GRIPPER_BEAM_BRAKE_ID);

    public GripperIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        motorsConfig.smartCurrentLimit(MOTORS_CURRENT_LIMIT);
        rightMotor.configure(motorsConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        leftMotor.configure(motorsConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Inputs:

    @Override
    protected boolean getIsCoralIn() {
        return beamBrake.get();
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

    @Override
    protected double getBackMotorVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getBackMotorVoltage'");
    }

    @Override
    protected double getLeftOuttakeMotorVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getLeftOuttakeMotorVoltage'");
    }

    @Override
    protected double getRightOuttakeMotorVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRightOuttakeMotorVoltage'");
    }
}
