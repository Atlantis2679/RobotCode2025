package frc.robot.subsystems.gripper.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.RobotMap.CANBUS;

import static frc.robot.RobotMap.*;
import static frc.robot.subsystems.gripper.GripperConstants.*;

public class GripperIOSparkMax extends GripperIO {
    private final SparkMax rightOuttakeMotor = new SparkMax(CANBUS.GRIPPER_RIGHT_OUTTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftOuttakeMotor = new SparkMax(CANBUS.GRIPPER_LEFT_OUTTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax backMotor = new SparkMax(CANBUS.GRIPPER_BACK_MOTOR_ID, MotorType.kBrushless);

    private final DigitalInput beamBrake = new DigitalInput(GRIPPER_BEAM_BRAKE_ID);

    private final SparkMaxConfig outtakeMotorsConfig = new SparkMaxConfig();
    private final SparkMaxConfig backMotorConfig = new SparkMaxConfig();

    private final REVLibError rightOuttakeMotorConfigError;
    private final REVLibError leftOuttakeMotorConfigError;
    private final REVLibError backMotorConfigError;

    public GripperIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);

        outtakeMotorsConfig.smartCurrentLimit(OUTTAKE_MOTORS_MAX_CURRENT);
        backMotorConfig.smartCurrentLimit(BACK_MOTOR_MAX_CURRENT);

        rightOuttakeMotorConfigError = rightOuttakeMotor.configure(outtakeMotorsConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        leftOuttakeMotorConfigError = leftOuttakeMotor.configure(outtakeMotorsConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        backMotorConfigError = backMotor.configure(backMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
    protected double getRightOuttakeMotorVoltage() {
        return rightOuttakeMotor.getAppliedOutput();
    }

    @Override
    protected double getRightOuttakeMotorCurrent() {
        return rightOuttakeMotor.getOutputCurrent();
    }

    @Override
    protected double getLeftOuttakeMotorVoltage() {
        return leftOuttakeMotor.getAppliedOutput();
    }

    @Override
    protected double getLeftOuttakeMotorCurrent() {
        return leftOuttakeMotor.getOutputCurrent();
    }

    @Override
    protected double getBackMotorVoltage() {
        return backMotor.getAppliedOutput();
    }

    @Override
    protected double getBackMotorCurrent() {
        return backMotor.getOutputCurrent();
    }

    @Override
    protected REVLibError getRightOuttakeMotorConfigError() {
        return rightOuttakeMotorConfigError;
    }

    @Override
    protected REVLibError getLeftOuttakeMotorConfigError() {
        return leftOuttakeMotorConfigError;
    }

    @Override
    protected REVLibError getBackMotorConfigError() {
        return backMotorConfigError;
    }

    @Override
    protected Faults getRightOuttakeMotorFaults() {
        return rightOuttakeMotor.getFaults();
    }

    @Override
    protected Faults getLeftOuttakeMotorFaults() {
        return leftOuttakeMotor.getFaults();
    }

    @Override
    protected Faults getBackMotorFaults() {
        return backMotor.getFaults();
    }

    @Override
    protected Warnings getRightOuttakeMotorWarnings() {
        return rightOuttakeMotor.getWarnings();
    }

    @Override
    protected Warnings getLeftOuttakeMotorWarnings() {
        return leftOuttakeMotor.getWarnings();
    }

    @Override
    protected Warnings getBackMotorWarnings() {
        return backMotor.getWarnings();
    }
}
