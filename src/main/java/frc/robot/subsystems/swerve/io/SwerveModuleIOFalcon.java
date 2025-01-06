package frc.robot.subsystems.swerve.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;

import static frc.robot.subsystems.swerve.SwerveContants.*;

import frc.lib.logfields.LogFieldsTable;

public class SwerveModuleIOFalcon extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder canCoder;

    private final PositionVoltage angleVoltagePositionControl = new PositionVoltage(0).withSlot(0);
    private final VoltageOut driveVoltageControl = new VoltageOut(0);
    private final DutyCycleOut drivePrecentageControl = new DutyCycleOut(0);
    private final StatusCode statusAngelMotor;

    private final Slot0Configs slot0ConfigsAngle;

    public SwerveModuleIOFalcon(LogFieldsTable fieldsTable, int driveMotorID, int angleMotorID, int encoderID) {
        super(fieldsTable);

        driveMotor = new TalonFX(driveMotorID);
        angleMotor = new TalonFX(angleMotorID);
        canCoder = new CANcoder(encoderID);

        // drive motor configs
        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfiguration.Feedback.SensorToMechanismRatio = GEAR_RATIO_DRIVE;
        driveMotorConfiguration.CurrentLimits.StatorCurrentLimit = 50;
        driveMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 35;
        driveMotorConfiguration.CurrentLimits.SupplyCurrentThreshold = 45;
        driveMotorConfiguration.CurrentLimits.SupplyTimeThreshold = 0.1;

        driveMotor.getVelocity().setUpdateFrequency(100);
        driveMotor.getPosition().setUpdateFrequency(100);
        driveMotor.getConfigurator().apply(driveMotorConfiguration);

        // angle motor configs
        TalonFXConfiguration angleMotorConfiguration = new TalonFXConfiguration();

        angleMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleMotorConfiguration.Feedback.SensorToMechanismRatio = GEAR_RATIO_ANGLE;
        angleMotorConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        angleMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        angleMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
        angleMotorConfiguration.CurrentLimits.SupplyCurrentThreshold = 30;
        angleMotorConfiguration.CurrentLimits.SupplyTimeThreshold = 0.1;
        angleMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        angleMotorConfiguration.CurrentLimits.StatorCurrentLimit = 30;

        slot0ConfigsAngle = angleMotorConfiguration.Slot0;
        slot0ConfigsAngle.kP = MODULE_ANGLE_KP;
        slot0ConfigsAngle.kI = MODULE_ANGLE_KI;
        slot0ConfigsAngle.kD = MODULE_ANGLE_KD;

        angleMotor.getPosition().setUpdateFrequency(100);
        statusAngelMotor = angleMotor.getConfigurator().apply(angleMotorConfiguration);

        // cancoder configs
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoder.getConfigurator().apply(canCoderConfiguration);
    }

    @Override
    protected double getAbsoluteAngleRotations() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    protected double getDriveSpeedRPS() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    @Override
    protected double getDriveMotorRotations() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    @Override
    protected double getIntegratedAngleEncoderRotations() {
        return angleMotor.getPosition().getValueAsDouble();
    }

    @Override
    protected double getP() {
        return slot0ConfigsAngle.kP;
    }

    @Override
    protected double getI() {
        return slot0ConfigsAngle.kI;
    }

    @Override
    protected double getD() {
        return slot0ConfigsAngle.kD;
    }

    @Override
    protected String getstatusAngelMotorName() {
        return statusAngelMotor.getName();
    }

    @Override
    public void setDriveSpeedPrecentage(double demandPrcentOutput) {
        driveMotor.setControl(drivePrecentageControl.withOutput(demandPrcentOutput));
    }

    @Override
    public void setDriveSpeedVoltage(double voltage) {
        driveMotor.setControl(driveVoltageControl.withOutput(voltage));
    }

    @Override
    public void setAngleMotorPositionRotations(double rotations) {
        angleMotor.setControl(angleVoltagePositionControl.withPosition(rotations));
    }

    @Override
    public void setIntegratedEncoderAngleEncoderRotations(double angleRotations) {
        angleMotor.setPosition(angleRotations);
    }

    @Override
    public void coastAll() {
        driveMotor.setControl(new CoastOut());
        angleMotor.setControl(new CoastOut());
    }

    @Override
    public void setP(double p) {
        slot0ConfigsAngle.kP = p;
        angleMotor.getConfigurator().apply(slot0ConfigsAngle);
    }

    @Override
    public void setI(double i) {
        slot0ConfigsAngle.kI = i;
        angleMotor.getConfigurator().apply(slot0ConfigsAngle);
    }

    @Override
    public void setD(double d) {
        slot0ConfigsAngle.kD = d;
        angleMotor.getConfigurator().apply(slot0ConfigsAngle);
    }
}