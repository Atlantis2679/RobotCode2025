package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class SwerveModuleIO extends IOBase {
        public final DoubleSupplier absoluteTurnAngleRotations = fields.addDouble("absoluteTurnAngleRotations",
                        this::getAbsoluteTurnAngleRotations);
        public final DoubleSupplier integratedTurnAngleRotations = fields.addDouble("integratedTurnAngleRotations",
                        this::getIntegratedTurnAngleRotations);
        public final DoubleSupplier driveSpeedRPS = fields.addDouble("driveSpeedRPS", this::getDriveSpeedRPS);
        public final DoubleSupplier driveDistanceRotations = fields.addDouble("driveDistanceRotations",
                        this::getDriveDistanceRotations);
        public final DoubleSupplier driveSupplyCurrent = fields.addDouble("driveSupplyCurrent",
                        this::getDriveSupplyCurrent);
        public final DoubleSupplier driveStatorCurrent = fields.addDouble("driveStatorCurrent",
                        this::getDriveStatorCurrent);
        public final DoubleSupplier turnSupplyCurrent = fields.addDouble("turnSupplyCurrent",
                        this::getTurnSupplyCurrent);
        public final DoubleSupplier turnStatorCurrent = fields.addDouble("turnStatorCurrent",
                        this::getTurnStatorCurrent);
        public final DoubleSupplier TurnKP = fields.addDouble("Turn kP", this::getTurnKP);
        public final DoubleSupplier TurnKI = fields.addDouble("Turn kI", this::getTurnKI);
        public final DoubleSupplier TurnKD = fields.addDouble("Turn kD", this::getTurnKD);
        public final DoubleSupplier driveMotorAcceleration = fields.addDouble("driveMotorAcceleration",
                        this::getDriveMotorAcceleration);
        public final DoubleSupplier turnMotorAcceleration = fields.addDouble("turnMotorAcceleration",
                        this::getTurnMotorAcceleration);
        public final DoubleSupplier driveMotorTemperature = fields.addDouble(
                        "driveMotorTemperature", this::getDriveMotorTemperature);
        public final DoubleSupplier turnMotorTemperature = fields.addDouble(
                        "turnMotorTemperature", this::getTurnMotorTemperature);
        public final Supplier<StatusCode> driveMotorConfigStatusCode = fields.addStatusCode(
                        "driveMotorConfigStatusCode", this::getDriveMotorConfigStatusCode);
        public final Supplier<StatusCode> driveMotorStatusCode = fields.addStatusCode(
                        "driveMotorStatusCode", this::getDriveMotorStatusCode);
        public final Supplier<StatusCode> turnMotorConfigStatusCode = fields.addStatusCode(
                        "turnMotorConfigStatusCode", this::getTurnMotorConfigStatusCode);
        public final Supplier<StatusCode> turnMotorStatusCode = fields.addStatusCode(
                        "turnMotorStatusCode", this::getTurnMotorStatusCode);
        public final Supplier<StatusCode> canCoderConfigStatusCode = fields.addStatusCode(
                        "canCoderConfigStatusCode", this::getCanCoderConfigStatusCode);
        public final Supplier<StatusCode> canCoderStatusCode = fields.addStatusCode(
                        "canCoderStatusCode", this::getCanCoderStatusCode);

        public SwerveModuleIO(LogFieldsTable fieldsTable) {
                super(fieldsTable);
        }

        // inputs

        protected abstract double getAbsoluteTurnAngleRotations();

        protected abstract double getDriveSpeedRPS();

        protected abstract double getIntegratedTurnAngleRotations();

        protected abstract double getDriveDistanceRotations();

        protected abstract double getDriveSupplyCurrent();

        protected abstract double getDriveStatorCurrent();

        protected abstract double getTurnSupplyCurrent();

        protected abstract double getTurnStatorCurrent();

        protected abstract double getTurnKP();

        protected abstract double getTurnKI();

        protected abstract double getTurnKD();

        protected abstract double getDriveMotorAcceleration();

        protected abstract double getTurnMotorAcceleration();

        // Network alerts Info:

        protected abstract double getDriveMotorTemperature();

        protected abstract double getTurnMotorTemperature();

        protected abstract StatusCode getDriveMotorStatusCode();

        protected abstract StatusCode getDriveMotorConfigStatusCode();

        protected abstract StatusCode getTurnMotorStatusCode();

        protected abstract StatusCode getTurnMotorConfigStatusCode();

        protected abstract StatusCode getCanCoderStatusCode();

        protected abstract StatusCode getCanCoderConfigStatusCode();

        // Outputs

        public abstract void setDriveSpeedPrecentage(double demand);

        public abstract void setDriveSpeedVoltage(double voltageDemand);

        public abstract void setTurnAngleRotations(double angleRotations);

        public abstract void resetIntegratedTurnAngleRotations(double angleRotations);

        public abstract void coastAll();

        public abstract void setTurnKP(double p);

        public abstract void setTurnKI(double I);

        public abstract void setTurnKD(double d);
}
