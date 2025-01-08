package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.logfields.IOBase;

public abstract class SwerveModuleIO extends IOBase {
    public final DoubleSupplier absoluteAngleRotations = fields.addDouble("absoluteAngleRotations",
            this::getAbsoluteAngleRotations);
    public final DoubleSupplier integratedEncoderAngleRotations = fields.addDouble("integratedEncoderAngleRotations",
            this::getIntegratedAngleEncoderRotations);
    public final DoubleSupplier driveSpeedRPS = fields.addDouble("driveSpeedRPS", this::getDriveSpeedRPS);
    public final DoubleSupplier driveMotorRotations = fields.addDouble("driveMotorRotations",
            this::getDriveMotorRotations);
    public final DoubleSupplier kP = fields.addDouble("kP", this::getP);
    public final DoubleSupplier kI = fields.addDouble("kI", this::getI);
    public final DoubleSupplier kD = fields.addDouble("kD", this::getD);

    public SwerveModuleIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputs

    protected abstract double getAbsoluteAngleRotations();

    protected abstract double getDriveSpeedRPS();

    protected abstract double getIntegratedAngleEncoderRotations();

    protected abstract double getDriveMotorRotations();

    protected abstract double getP();

    protected abstract double getI();

    protected abstract double getD();

    // Outputs

    public abstract void setDriveSpeedPrecentage(double demand);

    public abstract void setDriveSpeedVoltage(double voltageDemand);

    public abstract void setAngleMotorPositionRotations(double angle);

    public abstract void setIntegratedEncoderAngleEncoderRotations(double degrees);

    public abstract void coastAll();

    public abstract void setP(double p);

    public abstract void setI(double I);

    public abstract void setD(double d);
}
