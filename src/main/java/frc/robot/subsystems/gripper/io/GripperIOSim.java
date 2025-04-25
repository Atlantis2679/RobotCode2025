package frc.robot.subsystems.gripper.io;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkBase.Faults;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.networkalerts.GenericError;
import frc.robot.utils.GenericErrorGenerator;

public class GripperIOSim extends GripperIO {
    public GripperIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:

    @Override
    protected boolean getIsCoralIn() {
        return false;
    }

    // Outputs:

    @Override
    public void setBreakMotor(boolean isBreak) {
    }

    @Override
    public void setRightOuttakeMotorVoltage(double voltage) {
    }

    @Override
    public void setLeftOuttakeMotorVoltage(double voltage) {
    }

    @Override
    public void setBackMotorVoltage(double voltage) {
    }

    @Override
    protected double getRightOuttakeMotorCurrent() {
        return 0;
    }

    @Override
    protected double getLeftOuttakeMotorCurrent() {
        return 0;
    }

    @Override
    protected double getBackMotorCurrent() {
        return 0;
    }

    @Override
    protected GenericError getRightOuttakeMotorError() {
        return GenericErrorGenerator.revError(REVLibError.kOk, "Gripper", "Right Outtake Motor");
    }

    @Override
    protected GenericError getLeftOuttakeMotorError() {
        return GenericErrorGenerator.revError(REVLibError.kOk, "Gripper", "Left Outtake Motor");
    }

    @Override
    protected GenericError getBackMotorError() {
        return GenericErrorGenerator.revError(REVLibError.kOk, "Gripper", "Back Motor");
    }

    @Override
    protected GenericError getRightOuttakeMotorWarning() {
        return GenericErrorGenerator.sparkMaxWarning(new Warnings(0), "Gripper", "Right Outtake Motor");
    }

    @Override
    protected GenericError getLeftOuttakeMotorWarning() {
        return GenericErrorGenerator.sparkMaxWarning(new Warnings(0), "Gripper", "Left Outtake Motor");
    }

    @Override
    protected GenericError getBackMotorWarning() {
        return GenericErrorGenerator.sparkMaxWarning(new Warnings(0), "Gripper", "Back Motor");
    }

    @Override
    protected GenericError getRightOuttakeMotorConfigError() {
        return GenericErrorGenerator.revError(REVLibError.kOk, "Gripper", "Right Outtake Motor");
    }

    @Override
    protected GenericError getLeftOuttakeMotorConfigError() {
        return GenericErrorGenerator.revError(REVLibError.kOk, "Gripper", "Left Outtake Motor");
    }

    @Override
    protected GenericError getBackMotorConfigError() {
        return GenericErrorGenerator.revError(REVLibError.kOk, "Gripper", "Back Motor");
    }
}
