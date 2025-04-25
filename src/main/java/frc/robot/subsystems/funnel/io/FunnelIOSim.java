package frc.robot.subsystems.funnel.io;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.GenericError;
import frc.robot.utils.GenericErrorGenerator;

public class FunnelIOSim extends FunnelIO {
    public FunnelIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    public void setPercentageSpeed(double percentageSpeed) {
    }

    @Override
    protected boolean getIsCoralIn() {
        return false;
    }

    @Override
    protected double getCurrent() {
        return 0;
    }

    @Override
    protected GenericError getMotorError() {
        return GenericErrorGenerator.sparkMaxError(new Faults(0), "Funnel", "Motor");
    }

    @Override
    protected GenericError getMotorWarning() {
        return GenericErrorGenerator.sparkMaxWarning(new Warnings(0), "Funnel", "Motor");
    }

    @Override
    protected GenericError getMotorConfigError() {
        return GenericErrorGenerator.revError(REVLibError.kOk, "Funnel", "Motor");
    }
}