package frc.robot.subsystems.swerve.io;

import static frc.robot.subsystems.swerve.SwerveContants.GEAR_RATIO_DRIVE;
import static frc.robot.subsystems.swerve.SwerveContants.GEAR_RATIO_TURN;
import static frc.robot.subsystems.swerve.SwerveContants.MODULE_TURN_KD;
import static frc.robot.subsystems.swerve.SwerveContants.MODULE_TURN_KI;
import static frc.robot.subsystems.swerve.SwerveContants.MODULE_TURN_KP;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import frc.robot.utils.AlertsFactory;

import static frc.robot.subsystems.swerve.SwerveContants.MODULE_TEMPERATORE_WARNING_THRESHOLD;

public class SwerveModuleIOFalcon extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder canCoder;

    private final PositionVoltage turnPositionVoltageControl = new PositionVoltage(0).withSlot(0);
    private final VoltageOut driveVoltageControl = new VoltageOut(0);
    private final DutyCycleOut drivePrecentageControl = new DutyCycleOut(0);

    private final Slot0Configs turnSlotConfigs;

    public SwerveModuleIOFalcon(LogFieldsTable fieldsTable, int driveMotorID, int turnMotorID, int encoderID, int moduleNum) {
        super(fieldsTable);

        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);
        canCoder = new CANcoder(encoderID);

        // drive motor configs
        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();

        driveMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfiguration.Feedback.SensorToMechanismRatio = GEAR_RATIO_DRIVE;

        driveMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfiguration.CurrentLimits.StatorCurrentLimit = 90;

        driveMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 70;
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;

        driveMotor.getVelocity().setUpdateFrequency(100);
        driveMotor.getPosition().setUpdateFrequency(100);
        // turn motor configs
        TalonFXConfiguration turnMotorConfiguration = new TalonFXConfiguration();

        turnMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnMotorConfiguration.Feedback.SensorToMechanismRatio = GEAR_RATIO_TURN;
        turnMotorConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

        turnMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        turnMotorConfiguration.CurrentLimits.StatorCurrentLimit = 30;

        turnSlotConfigs = turnMotorConfiguration.Slot0;
        turnSlotConfigs.kP = MODULE_TURN_KP;
        turnSlotConfigs.kI = MODULE_TURN_KI;
        turnSlotConfigs.kD = MODULE_TURN_KD;

        turnMotor.getPosition().setUpdateFrequency(100);

        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();

        StatusCode driveMotorConfigError = driveMotor.getConfigurator().apply(driveMotorConfiguration);
        StatusCode turnMotorConfigError = turnMotor.getConfigurator().apply(turnMotorConfiguration);
        StatusCode canCoderConfigError = canCoder.getConfigurator().apply(canCoderConfiguration);

        AlertsFactory.phoenixMotor(PeriodicAlertsGroup.defaultInstance, "Swerve Module " + moduleNum + " Drive Motor Config: ",
            () -> driveMotorConfigError);
        
        AlertsFactory.phoenixMotor(PeriodicAlertsGroup.defaultInstance, "Swerve Module " + moduleNum + " Turn Motor Config: ", 
            () -> turnMotorConfigError);

        AlertsFactory.phoenixMotor(PeriodicAlertsGroup.defaultInstance, "Swerve Module " + moduleNum + " Can Coder Config: ", 
            () -> canCoderConfigError);

        PeriodicAlertsGroup.defaultInstance.addWarningAlert(
            () -> "Swerve Module " + moduleNum + " Drive Motor Temp: " + driveMotor.getDeviceTemp().getValueAsDouble(),
            () -> {return driveMotor.getDeviceTemp().getValueAsDouble() > MODULE_TEMPERATORE_WARNING_THRESHOLD;});

        PeriodicAlertsGroup.defaultInstance.addWarningAlert(
            () -> "Swerve Module " + moduleNum + " Turn Motor Temp: " + turnMotor.getDeviceTemp().getValueAsDouble(),
            () -> {return turnMotor.getDeviceTemp().getValueAsDouble() > MODULE_TEMPERATORE_WARNING_THRESHOLD;});
    }

    @Override
    protected double getAbsoluteTurnAngleRotations() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    protected double getDriveSpeedRPS() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    @Override
    protected double getDriveDistanceRotations() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    @Override
    protected double getIntegratedTurnAngleRotations() {
        return turnMotor.getPosition().getValueAsDouble();
    }

    @Override
    protected double getTurnKP() {
        return turnSlotConfigs.kP;
    }

    @Override
    protected double getTurnKI() {
        return turnSlotConfigs.kI;
    }

    @Override
    protected double getTurnKD() {
        return turnSlotConfigs.kD;
    }

    @Override
    public void setDriveSpeedPrecentage(double demand) {
        driveMotor.setControl(drivePrecentageControl.withOutput(demand));
    }

    @Override
    public void setDriveSpeedVoltage(double demandVoltage) {
        driveMotor.setControl(driveVoltageControl.withOutput(demandVoltage));
    }

    @Override
    public void setTurnAngleRotations(double angleRotations) {
        turnMotor.setControl(turnPositionVoltageControl.withPosition(angleRotations));
    }

    @Override
    public void resetIntegratedTurnAngleRotations(double angleRotations) {
        turnMotor.setPosition(angleRotations);
    }

    @Override
    public void coastAll() {
        driveMotor.setControl(new CoastOut());
        turnMotor.setControl(new CoastOut());
    }

    @Override
    public void setTurnKP(double p) {
        turnSlotConfigs.kP = p;
        turnMotor.getConfigurator().apply(turnSlotConfigs);
    }

    @Override
    public void setTurnKI(double i) {
        turnSlotConfigs.kI = i;
        turnMotor.getConfigurator().apply(turnSlotConfigs);
    }

    @Override
    public void setTurnKD(double d) {
        turnSlotConfigs.kD = d;
        turnMotor.getConfigurator().apply(turnSlotConfigs);
    }

    @Override
    protected double getDriveSupplyCurrent() {
        return driveMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    protected double getDriveStatorCurrent() {
        return driveMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    protected double getTurnSupplyCurrent() {
        return turnMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    protected double getTurnStatorCurrent() {
        return turnMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    protected double getDriveMotorTemperature() {
        return driveMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    protected double getTurnMotorTemperature() {
        return turnMotor.getDeviceTemp().getValueAsDouble();
    }
}