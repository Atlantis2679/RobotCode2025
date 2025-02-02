package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.logfields.IOBase;

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
        public final Supplier<StatusCode> driveStatusCode = fields.addStatusCode("driveStatusCode", this::getDriveStatusCode);
        public final Supplier<StatusCode> turnStatusCode = fields.addStatusCode("turnStatusCode", this::getTurnStatusCode);
        public final Supplier<StatusCode> canCoderStatusCode = fields.addStatusCode("canCoderStatusCode", this::getCanCoderStatusCode);
        public final Supplier<StatusCode> driveConfigurationStatusCode = fields.addStatusCode("driveConfigurationStatusCode", this::getDriveConfigurationStatusCode);
        public final Supplier<StatusCode> turnConfigurationStatusCode = fields.addStatusCode("turnConfigurationStatusCode", this::getTurnConfigurationStatusCode);
        public final Supplier<StatusCode> canCoderConfigurationStatusCode = fields.addStatusCode("canCoderConfigurationStatusCode", this::getCanCoderConfigurationStatusCode);
        public final DoubleSupplier TurnKP = fields.addDouble("Turn kP", this::getTurnKP);
        public final DoubleSupplier TurnKI = fields.addDouble("Turn kI", this::getTurnKI);
        public final DoubleSupplier TurnKD = fields.addDouble("Turn kD", this::getTurnKD);

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

        protected abstract StatusCode getDriveStatusCode();

        protected abstract StatusCode getTurnStatusCode();

        protected abstract StatusCode getCanCoderStatusCode();

        protected abstract StatusCode getDriveConfigurationStatusCode();

        protected abstract StatusCode getTurnConfigurationStatusCode();

        protected abstract StatusCode getCanCoderConfigurationStatusCode();

        protected abstract double getTurnKP();

        protected abstract double getTurnKI();

        protected abstract double getTurnKD();

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
