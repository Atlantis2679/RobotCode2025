package frc.robot.subsystems.gripper.io;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.RobotMap.CANBUS;
import frc.robot.utils.NetworkAlertsMotors;

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

        rightOuttakeMotorConfig.idleMode(IdleMode.kCoast);
        leftOuttakeMotorConfig.idleMode(IdleMode.kCoast);
        backMotorConfig.idleMode(IdleMode.kCoast);

        leftOuttakeMotorConfig.inverted(true);

        REVLibError leftOuttakeMotorConfigError = leftOuttakeMotor.configure(leftOuttakeMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        REVLibError rightOuttakeMotorConfigError = rightOuttakeMotor.configure(rightOuttakeMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        REVLibError backMotorConfigError = backMotor.configure(backMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        NetworkAlertsMotors.addRevLibErrorAlert("Gripper Left Motor Config: ", () -> leftOuttakeMotorConfigError);
        NetworkAlertsMotors.addRevLibErrorAlert("Gripper Left Motor Config: ", () -> rightOuttakeMotorConfigError);
        NetworkAlertsMotors.addRevLibErrorAlert("Gripper Left Motor Config: ", () -> backMotorConfigError);

        NetworkAlertsMotors.addMotorStuckAlert("Gripper Left Motor is Stuck!", leftOuttakeMotorCurrent,
                leftOuttakeMotor::getAppliedOutput);
        NetworkAlertsMotors.addMotorStuckAlert("Gripper Right Motor is Stuck!", rightOuttakeMotorCurrent,
                rightOuttakeMotor::getAppliedOutput);
        NetworkAlertsMotors.addMotorStuckAlert("Gripper Back Motor is Stuck!", backMotorCurrent,
                backMotor::getAppliedOutput);

        NetworkAlertsMotors.addSparkMotorAlert("Gripper Left Motor: ", leftOuttakeMotor::getFaults,
                leftOuttakeMotor::getWarnings);
        NetworkAlertsMotors.addSparkMotorAlert("Gripper Right Motor: ", rightOuttakeMotor::getFaults,
                rightOuttakeMotor::getWarnings);
        NetworkAlertsMotors.addSparkMotorAlert("Gripper Back Motor: ", backMotor::getFaults, backMotor::getWarnings);
    }

    // Outputs:

    @Override
    public void setBreakMotor(boolean isBreak) {
        rightOuttakeMotorConfig.idleMode(isBreak ? IdleMode.kBrake : IdleMode.kCoast);
        leftOuttakeMotorConfig.idleMode(isBreak ? IdleMode.kBrake : IdleMode.kCoast);
        backMotorConfig.idleMode(isBreak ? IdleMode.kBrake : IdleMode.kCoast);
        
        rightOuttakeMotor.configure(rightOuttakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        leftOuttakeMotor.configure(leftOuttakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        backMotor.configure(backMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

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
