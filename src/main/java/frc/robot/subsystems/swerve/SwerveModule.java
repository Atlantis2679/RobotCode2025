package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.robot.Robot;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOFalcon;
import frc.robot.subsystems.swerve.io.SwerveModuleIOSim;
import frc.robot.utils.PrimitiveRotationalSensorHelper;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.swerve.SwerveContants.*;

public class SwerveModule implements Tuneable {
    private final int moduleNumber;
    private final String positionName;

    private final LogFieldsTable fieldsTable;
    private final SwerveModuleIO io;

    private PrimitiveRotationalSensorHelper absoluteAngleHelperDegrees;

    private double lastDriveDistanceMeters;
    private double currDriveDistanceMeters;
    private boolean encoderResetToAbsoluteQueued = false;

    private final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;

    public SwerveModule(int moduleNumber, String positionName, int driveMotorID, int turnMotorID, int encoderID,
            double absoluteAngleOffSetDegrees, LogFieldsTable swerveFieldsTable) {
        this.moduleNumber = moduleNumber;
        this.positionName = positionName;

        fieldsTable = swerveFieldsTable.getSubTable("Module " + moduleNumber + " " + positionName);

        io = Robot.isSimulation()
                ? new SwerveModuleIOSim(fieldsTable, driveMotorID, turnMotorID, encoderID,
                        absoluteAngleOffSetDegrees)
                : new SwerveModuleIOFalcon(fieldsTable, driveMotorID, turnMotorID, encoderID);

        fieldsTable.update();

        absoluteAngleHelperDegrees = new PrimitiveRotationalSensorHelper(
                io.absoluteTurnAngleRotations.getAsDouble() * 360,
                absoluteAngleOffSetDegrees);

        lastDriveDistanceMeters = getDriveDistanceMeters();
        currDriveDistanceMeters = getDriveDistanceMeters();

        io.resetIntegratedTurnAngleRotations(getAbsoluteAngleDegrees() / 360);
    }

    public void periodic() {
        absoluteAngleHelperDegrees.update(io.absoluteTurnAngleRotations.getAsDouble() * 360);
        lastDriveDistanceMeters = currDriveDistanceMeters;
        currDriveDistanceMeters = getDriveDistanceMeters();
        fieldsTable.recordOutput("module " + moduleNumber +" drive distance meters", getDriveDistanceMeters());
        fieldsTable.recordOutput("AbsoluteAngleDegrees", getAbsoluteAngleDegrees());
        fieldsTable.recordOutput("IntegratedAngleDegrees", getIntegratedAngleDegrees());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean preventJittering, boolean optimizeState,
            boolean useVoltage) {
        if (preventJittering && Math.abs(desiredState.speedMetersPerSecond) < MAX_MODULE_VELOCITY_MPS * 0.01) {
            io.setDriveSpeedPrecentage(0);
            return;
        }

        final double currentAngleDegrees;

        if (encoderResetToAbsoluteQueued) {
            io.resetIntegratedTurnAngleRotations(getAbsoluteAngleDegrees() / 360);
            currentAngleDegrees = getAbsoluteAngleDegrees();
            encoderResetToAbsoluteQueued = false;
        } else {
            currentAngleDegrees = getIntegratedAngleDegrees();
        }

        if (optimizeState) {
            desiredState.optimize(Rotation2d.fromDegrees(currentAngleDegrees));
        }

        if (useVoltage) {
            io.setDriveSpeedVoltage((desiredState.speedMetersPerSecond / MAX_MODULE_VELOCITY_MPS) * MAX_VOLTAGE);
        } else {
            io.setDriveSpeedPrecentage(desiredState.speedMetersPerSecond / MAX_MODULE_VELOCITY_MPS);
        }
        io.setTurnAngleRotations(desiredState.angle.getRotations());
    }
    
    public void setDesiredStateVoltage(Voltage voltage) {
        //Multipling MAX_MODULE_VELOCITY_MPS and deviding by MAX_VOLTAGE to get a pure voltage result
        setDesiredState(new SwerveModuleState((voltage.in(Volts)*MAX_MODULE_VELOCITY_MPS)/MAX_VOLTAGE, new Rotation2d()), true, true, true);
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

    public String getName() {
        return this.positionName;
    }

    public double getDriveDistanceMeters() {
        return io.driveDistanceRotations.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getIntegratedAngleDegrees() {
        return io.integratedTurnAngleRotations.getAsDouble() * 360;
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocityMPS(), getRotation2d());
    }

    public SwerveModuleState getModuleStateIntegrated() {
        return new SwerveModuleState(getVelocityMPS(), Rotation2d.fromDegrees(getIntegratedAngleDegrees()));
    }

    public double getVelocityMPS() {
        return io.driveSpeedRPS.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAbsoluteAngleDegrees());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getRotation2d());
    }

    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(
                getDriveDistanceMeters() - lastDriveDistanceMeters,
                getRotation2d());
    }

    public void resetAbsoluteAngleDegrees(double degrees) {
        absoluteAngleHelperDegrees.resetAngle(degrees);
        queueResetToAbsolute();
    }

    public double getSupplyVoltage(){
        return io.voltage.getAsDouble();
    }

    public double getTurnKP() {
        return io.TurnKP.getAsDouble();
    }

    public double getTurnKI() {
        return io.TurnKI.getAsDouble();
    }

    public double getTurnKD() {
        return io.TurnKD.getAsDouble();
    }

    public void setTurnKP(double p) {
        io.setTurnKP(p);
    }

    public void setTurnKI(double i) {
        io.setTurnKI(i);
    }

    public void setTurnKD(double d) {
        io.setTurnKD(d);
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.addDoubleProperty("Integrated Angle Degrees", this::getIntegratedAngleDegrees, null);
        builder.addDoubleProperty("Absolute Angle Degrees", this::getAbsoluteAngleDegrees, null);
        builder.addDoubleProperty("Tuneable Offset",
                absoluteAngleHelperDegrees::getOffset,
                val -> {
                    absoluteAngleHelperDegrees.setOffset(val);
                    queueResetToAbsolute();
                });
    }
    public void driveMotorUpdateLog(SysIdRoutineLog log) {
        log.motor("Module" + moduleNumber + "Drive")
                .angularPosition(Units.Rotations.of(getAbsoluteAngleDegrees()))
                .angularVelocity(Units.RotationsPerSecond.of(getVelocityMPS()))
                .voltage(Units.Volts.of(getSupplyVoltage()));

    }
}
