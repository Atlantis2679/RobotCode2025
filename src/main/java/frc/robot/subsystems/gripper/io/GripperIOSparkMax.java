package frc.robot.subsystems.gripper.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.RobotMap.CANBUS;

import static frc.robot.RobotMap.*;
import static frc.robot.subsystems.gripper.GripperConstants.*;

public class GripperIOSparkMax extends GripperIO {
    private final SparkMax rightOuttakeMotor = new SparkMax(CANBUS.GRIPPER_RIGHT_OUTTAKE_MOTOR_ID,
            MotorType.kBrushless);
    private final SparkMax leftOuttakeMotor = new SparkMax(CANBUS.GRIPPER_LEFT_OUTTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax backMotor = new SparkMax(CANBUS.GRIPPER_BACK_MOTOR_ID, MotorType.kBrushless);

    private final DigitalInput beamBrake = new DigitalInput(GRIPPER_BEAM_BRAKE_ID);

    private final SparkMaxConfig leftOuttakeMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightOuttakeMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig backMotorConfig = new SparkMaxConfig();

    public GripperIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);

        rightOuttakeMotorConfig.smartCurrentLimit(OUTTAKE_MOTORS_MAX_CURRENT);
        leftOuttakeMotorConfig.smartCurrentLimit(OUTTAKE_MOTORS_MAX_CURRENT);
        backMotorConfig.smartCurrentLimit(BACK_MOTOR_MAX_CURRENT);

        rightOuttakeMotorConfig.inverted(true);
        
        leftOuttakeMotor.configure(leftOuttakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        rightOuttakeMotor.configure(rightOuttakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        rightOuttakeMotor.configure(rightOuttakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    // Outputs:

    @Override
    public void setRightOuttakeMotorVoltage(double voltage) {
        rightOuttakeMotor.setVoltage(voltage);
    }

    @Override
    public void setLeftOuttakeMotorVoltage(double voltage) {
        leftOuttakeMotor.setVoltage(voltage);
    }

    @Override
    public void setBackMotorVoltage(double voltage) {
        backMotor.setVoltage(voltage);
    }

    // Inputs:

    @Override
    protected boolean getIsCoralIn() {
        return !beamBrake.get();
    }

    @Override
    protected double getRightOuttakeMotorCurrent() {
        return rightOuttakeMotor.getOutputCurrent();
    }

    @Override
    protected double getLeftOuttakeMotorCurrent() {
        return leftOuttakeMotor.getOutputCurrent();
    }

    @Override
    protected double getBackMotorCurrent() {
        return backMotor.getOutputCurrent();
    }
}
