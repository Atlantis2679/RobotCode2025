package frc.robot.subsystems.swerve.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.logfields.LogFieldsTable;
import static frc.robot.subsystems.swerve.SwerveContants.*;

public class SwerveModuleIOSim extends SwerveModuleIO {
    private final FlywheelSim driveMotorSim;
    private final FlywheelSim angleMotorSim;
    private double simEncoderAbsoluteRotations;
    private double simEncoderIntegeratedRotations = 0;
    private double simDriveRotations = 0;
    private final PIDController pidControllerAngle = new PIDController(15, 0, 0);

    public SwerveModuleIOSim(LogFieldsTable fieldsTable, int driveMotorID, int angleMotorID, int encoderID,
            double absoluteAngleOffsetDegrees) {
        super(fieldsTable);

        simEncoderAbsoluteRotations = calculateAbsolute(absoluteAngleOffsetDegrees / 360);

        driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), GEAR_RATIO_DRIVE, 0.05);
        angleMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), GEAR_RATIO_ANGLE, 0.004);
        pidControllerAngle.enableContinuousInput(-0.5, 0.5);
    }

    @Override
    public void periodicBeforeFields() {
        double angleRPS = angleMotorSim.getAngularVelocityRPM() / 60;
        double angleDiffRotations = angleRPS * 0.02;
        simEncoderIntegeratedRotations += angleDiffRotations;
        simEncoderAbsoluteRotations = calculateAbsolute(simEncoderAbsoluteRotations + angleDiffRotations);

        driveMotorSim.update(0.02);
        angleMotorSim.update(0.02);

        double driveRPS = driveMotorSim.getAngularVelocityRPM() / 60;
        double driveRotationsDiff = driveRPS * 0.02;
        simDriveRotations += driveRotationsDiff;

        double anglePIDResult = pidControllerAngle.calculate(calculateAbsolute(simEncoderIntegeratedRotations));
        angleMotorSim.setInputVoltage(anglePIDResult);
    }

    private final double calculateAbsolute(double rotations) {
        while (rotations > 0.5) {
            rotations -= 1;
        }

        while (rotations < 0.5) {
            rotations += 1;
        }

        return rotations;
    }

    @Override
    protected double getAbsoluteAngleRotations() {
        return simEncoderAbsoluteRotations;
    }

    @Override
    protected double getDriveSpeedRPS() {
        return driveMotorSim.getAngularVelocityRPM() / 60;
    }

    @Override
    protected double getIntegratedAngleEncoderRotations() {
        return simEncoderIntegeratedRotations;
    }

    @Override
    protected double getDriveMotorRotations() {
        return simDriveRotations;
    }

    @Override
    protected double getP() {
        return pidControllerAngle.getP();
    }

    @Override
    protected double getI() {
        return pidControllerAngle.getI();
    }

    @Override
    protected double getD() {
        return pidControllerAngle.getD();
    }

    @Override
    public void setDriveSpeedPrecentage(double demandPrcentOutput) {
        demandPrcentOutput = MathUtil.clamp(demandPrcentOutput, -1, 1);
        driveMotorSim.setInputVoltage(demandPrcentOutput * 12);
    }

    @Override
    public void setDriveSpeedVoltage(double voltageDemand) {
        voltageDemand = MathUtil.clamp(voltageDemand, -12, 12);
        driveMotorSim.setInputVoltage(voltageDemand);
    }

    @Override
    public void setAngleMotorPositionRotations(double rotations) {
        pidControllerAngle.setSetpoint(calculateAbsolute(rotations));
    }

    @Override
    public void setIntegratedEncoderAngleEncoderRotations(double angleRotations) {
        simEncoderIntegeratedRotations = angleRotations;
    }

    @Override
    public void coastAll() {
    }

    @Override
    public void setP(double p) {
        pidControllerAngle.setP(p);
    }

    @Override
    public void setI(double i) {
        pidControllerAngle.setI(i);
    }

    @Override
    public void setD(double d) {
        pidControllerAngle.setD(d);
    }

    @Override
    protected String getstatusAngelMotorName() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getstatusAngelMotorName'");
    }

}
