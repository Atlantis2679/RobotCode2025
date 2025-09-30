package frc.robot.subsystems.gripper.io;

import static frc.robot.RobotMap.GRIPPER_BEAM_BRAKE_ID;
import static frc.robot.subsystems.gripper.GripperConstants.BACK_MOTOR_MAX_CURRENT;
import static frc.robot.subsystems.gripper.GripperConstants.OUTTAKE_MOTORS_MAX_CURRENT;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import frc.robot.RobotMap.CANBUS;
import frc.robot.utils.AlertsFactory;

public class GripperIOSparkMax extends GripperIO {
    private final SparkMax rightMotor = new SparkMax(CANBUS.GRIPPER_RIGHT_OUTTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(CANBUS.GRIPPER_LEFT_OUTTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax backMotor = new SparkMax(CANBUS.GRIPPER_BACK_MOTOR_ID, MotorType.kBrushless);

    private final DigitalInput beamBrake = new DigitalInput(GRIPPER_BEAM_BRAKE_ID);

    private final SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig backMotorConfig = new SparkMaxConfig();

    private REVLibError leftMotorConfigError;
    private REVLibError rightMotorConfigError;
    private REVLibError backMotorConfigError;

    public GripperIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);

        rightMotorConfig.smartCurrentLimit(OUTTAKE_MOTORS_MAX_CURRENT);
        leftMotorConfig.smartCurrentLimit(OUTTAKE_MOTORS_MAX_CURRENT);
        backMotorConfig.smartCurrentLimit(BACK_MOTOR_MAX_CURRENT);

        rightMotorConfig.idleMode(IdleMode.kCoast);
        leftMotorConfig.idleMode(IdleMode.kCoast);
        backMotorConfig.idleMode(IdleMode.kCoast);

        leftMotorConfig.inverted(true);

        leftMotorConfigError = leftMotor.configure(leftMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        rightMotorConfigError = rightMotor.configure(rightMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        backMotorConfigError = backMotor.configure(backMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance, "Gripper: Left Motor config",
            () -> leftMotorConfigError , leftMotor::getWarnings, leftMotor::getFaults);
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance, "Gripper: Right Motor config",
            () -> rightMotorConfigError , rightMotor::getWarnings, rightMotor::getFaults);
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance, "Gripper: Back Motor config",
            () -> backMotorConfigError , backMotor::getWarnings, backMotor::getFaults);

    }

    // Outputs:

    @Override
    public void setBreakMotor(boolean isBreak) {
        rightMotorConfig.idleMode(isBreak ? IdleMode.kBrake : IdleMode.kCoast);
        leftMotorConfig.idleMode(isBreak ? IdleMode.kBrake : IdleMode.kCoast);
        backMotorConfig.idleMode(isBreak ? IdleMode.kBrake : IdleMode.kCoast);
        
        leftMotorConfigError = leftMotor.configure(leftMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        rightMotorConfigError = rightMotor.configure(rightMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        backMotorConfigError = backMotor.configure(backMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void setRightMotorVoltage(double voltage) {
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void setLeftMotorVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
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
    protected double getRightMotorCurrent() {
        return rightMotor.getOutputCurrent();
    }

    @Override
    protected double getLeftMotorCurrent() {
        return leftMotor.getOutputCurrent();
    }

    @Override
    protected double getBackMotorCurrent() {
        return backMotor.getOutputCurrent();
    }
}
