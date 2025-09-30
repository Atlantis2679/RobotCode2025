package frc.robot.subsystems.swerve.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import team2679.atlantiskit.logfields.LogFieldsTable;

import static frc.robot.subsystems.pivot.PivotConstants.KP;
import static frc.robot.subsystems.swerve.SwerveContants.*;

public class SwerveModuleIOSim extends SwerveModuleIO {
    private final FlywheelSim driveMotorSim;
    private final FlywheelSim turnMotorSim;
    private double simAbsoluteTurnRotations;
    private double simIntegeratedTurnRotations = 0;
    private double simDriveRotations = 0;
    private final PIDController turnPIDController = new PIDController(KP, 0, 0);

    public SwerveModuleIOSim(LogFieldsTable fieldsTable, int driveMotorID, int angleMotorID, int encoderID,
            double absoluteAngleOffsetDegrees) {
        super(fieldsTable);

        simAbsoluteTurnRotations = calculateToAbsoluteRotations(absoluteAngleOffsetDegrees / 360);

        DCMotor driveMotorModel = DCMotor.getFalcon500(1);
        DCMotor turnMotorModel = DCMotor.getFalcon500(1);

        driveMotorSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(driveMotorModel, 0.025, GEAR_RATIO_DRIVE),
                driveMotorModel);
        turnMotorSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(turnMotorModel, 0.004, GEAR_RATIO_TURN),
                turnMotorModel);
        turnPIDController.enableContinuousInput(-0.5, 0.5);
    }

    @Override
    public void periodicBeforeFields() {
        driveMotorSim.update(0.02);
        turnMotorSim.update(0.02);

        double turnVelocityRPS = turnMotorSim.getAngularVelocityRPM() / 60;
        double angleDiffRotations = turnVelocityRPS * 0.02;
        simIntegeratedTurnRotations += angleDiffRotations;
        simAbsoluteTurnRotations = calculateToAbsoluteRotations(
                simAbsoluteTurnRotations + angleDiffRotations);

        // Sim note - RPM stands for Radiands, not Rotations. Can be an incorrect use
        double driveVelocityRPS = driveMotorSim.getAngularVelocityRPM() / 60;
        double driveRotationsDiff = driveVelocityRPS * 0.02;
        simDriveRotations += driveRotationsDiff;

        double turnPIDResult = turnPIDController
                .calculate(calculateToAbsoluteRotations(simIntegeratedTurnRotations));
        turnMotorSim.setInputVoltage(turnPIDResult);
    }

    private final double calculateToAbsoluteRotations(double turnAngleRotations) {
        while (turnAngleRotations > 0.5) {
            turnAngleRotations -= 1;
        }

        while (turnAngleRotations < -0.5) {
            turnAngleRotations += 1;
        }

        return turnAngleRotations;
    }

    @Override
    protected double getAbsoluteTurnAngleRotations() {
        return simAbsoluteTurnRotations;
    }

    @Override
    protected double getDriveSpeedRPS() {
        return driveMotorSim.getAngularVelocityRPM() / 60;
    }

    @Override
    protected double getIntegratedTurnAngleRotations() {
        return simIntegeratedTurnRotations;
    }

    @Override
    protected double getDriveDistanceRotations() {
        return simDriveRotations;
    }

    @Override
    protected double getTurnKP() {
        return turnPIDController.getP();
    }

    @Override
    protected double getTurnKI() {
        return turnPIDController.getI();
    }

    @Override
    protected double getTurnKD() {
        return turnPIDController.getD();
    }

    @Override
    public void setDriveSpeedPrecentage(double demand) {
        demand = MathUtil.clamp(demand, -1, 1);
        driveMotorSim.setInputVoltage(demand * 12);
    }

    @Override
    public void setDriveSpeedVoltage(double voltageDemand) {
        voltageDemand = MathUtil.clamp(voltageDemand, -12, 12);
        driveMotorSim.setInputVoltage(voltageDemand);
    }

    @Override
    public void setTurnAngleRotations(double angleRotations) {
        turnPIDController.setSetpoint(calculateToAbsoluteRotations(angleRotations));
    }

    @Override
    public void resetIntegratedTurnAngleRotations(double angleRotations) {
        simIntegeratedTurnRotations = angleRotations;
    }

    @Override
    public void coastAll() {
        driveMotorSim.setInputVoltage(0);
    }

    @Override
    public void setTurnKP(double p) {
        turnPIDController.setP(p);
    }

    @Override
    public void setTurnKI(double i) {
        turnPIDController.setI(i);
    }

    @Override
    public void setTurnKD(double d) {
        turnPIDController.setD(d);
    }

    @Override
    protected double getDriveSupplyCurrent() {
        return driveMotorSim.getCurrentDrawAmps();
    }

    @Override
    protected double getDriveStatorCurrent() {
        return driveMotorSim.getCurrentDrawAmps();
    }

    @Override
    protected double getTurnSupplyCurrent() {
        return turnMotorSim.getCurrentDrawAmps();
    }

    @Override
    protected double getTurnStatorCurrent() {
        return turnMotorSim.getCurrentDrawAmps();
    }
}
