package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOFalcon;
import frc.robot.subsystems.swerve.io.SwerveModuleIOSim;
import frc.robot.utils.PrimitiveRotationalSensorHelper;

import static frc.robot.subsystems.swerve.SwerveContants.*;

public class SwerveModule implements Tuneable {
    private final int moduleNumber;

    private final LogFieldsTable fieldsTable;
    private final SwerveModuleIO io;

    private final int driveMotorID;
    private final int angleMotorID;
    private final int encoderID;
    private PrimitiveRotationalSensorHelper absoluteAngleHelperDegrees;

    private double lastDriveDistanceMeters;
    private double currDriveDistanceMeters;
    private boolean encoderResetToAbsoluteQueued = false;

    private final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int encoderID,
            double absoluteAngleOffSetDegrees, LogFieldsTable swerveFieldsTable) {
        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.encoderID = encoderID;

        fieldsTable = swerveFieldsTable.getSubTable("Module " + moduleNumber);

        io = Robot.isSimulation()
                ? new SwerveModuleIOSim(fieldsTable, this.driveMotorID, this.angleMotorID, this.encoderID,
                        absoluteAngleOffSetDegrees)
                : new SwerveModuleIOFalcon(fieldsTable, this.driveMotorID, this.angleMotorID, this.encoderID);

        fieldsTable.update();

        absoluteAngleHelperDegrees = new PrimitiveRotationalSensorHelper(
                io.absoluteAngleRotations.getAsDouble() * 360,
                absoluteAngleOffSetDegrees);

        lastDriveDistanceMeters = getDriveDistanceMeters();
        currDriveDistanceMeters = getDriveDistanceMeters();

        io.setIntegratedEncoderAngleEncoderRotations(getAbsoluteAngleDegrees() / 360);
    }

    public void periodic() {
        absoluteAngleHelperDegrees.update(io.absoluteAngleRotations.getAsDouble() * 360);
        lastDriveDistanceMeters = currDriveDistanceMeters;
        currDriveDistanceMeters = getDriveDistanceMeters();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean preventJittering, boolean optimizeState,
            boolean useVoltage) {
        if (preventJittering && Math.abs(desiredState.speedMetersPerSecond) < MAX_MODULE_SPEED_MPS * 0.01) {
            io.setDriveSpeedPrecentage(0);
            return;
        }

        final double currentAngleDegrees;

        if (encoderResetToAbsoluteQueued) {
            io.setIntegratedEncoderAngleEncoderRotations(getAbsoluteAngleDegrees() / 360);
            currentAngleDegrees = getAbsoluteAngleDegrees();
            encoderResetToAbsoluteQueued = false;
        } else {
            currentAngleDegrees = getIntegratedEncoderAngleDegrees();
        }

        if (optimizeState) {
            desiredState = SwerveModuleState.optimize(
                    desiredState,
                    Rotation2d.fromDegrees(currentAngleDegrees));
        }

        if (useVoltage) {
            io.setDriveSpeedVoltage((desiredState.speedMetersPerSecond / MAX_MODULE_SPEED_MPS) * MAX_VOLTAGE);
        } else {
            io.setDriveSpeedPrecentage(desiredState.speedMetersPerSecond / MAX_MODULE_SPEED_MPS);
        }
        io.setAngleMotorPositionRotations(desiredState.angle.getRotations());
    }

    public void queueResetToAbsolute() {
        encoderResetToAbsoluteQueued = true;
    }

    public void enableCoastMode() {
        io.coastAll();
    }

    public double getAbsoluteAngleDegrees() {
        return absoluteAngleHelperDegrees.getAngle();
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public double getDriveDistanceMeters() {
        return io.driveMotorRotations.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getIntegratedEncoderAngleDegrees() {
        return io.integratedEncoderAngleRotations.getAsDouble() * 360;
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleMPS(), getRotation2d());
    }

    public SwerveModuleState getModuleStateIntegreated() {
        return new SwerveModuleState(getModuleMPS(), Rotation2d.fromDegrees(getIntegratedEncoderAngleDegrees()));
    }

    public double getModuleMPS() {
        return io.driveSpeedRPS.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAbsoluteAngleDegrees());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(currDriveDistanceMeters, getRotation2d());
    }

    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(
                currDriveDistanceMeters - lastDriveDistanceMeters,
                getRotation2d());
    }

    public void setAbsoluteEncoderAngleDegrees(double degrees) {
        absoluteAngleHelperDegrees.resetAngle(degrees);
        queueResetToAbsolute();
    }

    public double getP() {
        return io.kP.getAsDouble();
    }

    public double getI() {
        return io.kI.getAsDouble();
    }

    public double getD() {
        return io.kD.getAsDouble();
    }

    public void setP(double p) {
        io.setP(p);
    }

    public void setI(double i) {
        io.setI(i);
    }

    public void setD(double d) {
        io.setD(d);
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.addDoubleProperty("Integrated Angle Degrees", this::getIntegratedEncoderAngleDegrees, null);
        builder.addDoubleProperty("Absolute Angle Degrees", this::getAbsoluteAngleDegrees, null);
        builder.addDoubleProperty("Tuneable Offset",
                absoluteAngleHelperDegrees::getOffset,
                val -> {
                    absoluteAngleHelperDegrees.setOffset(val);
                    ;
                    queueResetToAbsolute();
                });
    }
}
