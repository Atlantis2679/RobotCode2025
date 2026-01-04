package frc.robot.subsystems.elevator.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class ElevatorIO extends IOBase {
    public final DoubleSupplier rightMotorCurrent = fields.addDouble("rightNotorCurrect",
            this::getRightMotorCurrent);
    public final DoubleSupplier leftMotorCurrent = fields.addDouble("leftMotorCurrect",
            this::getLeftMotorCurrent);
    public final DoubleSupplier height = fields.addDouble("elevatorHeight", this::getHeightMeters);
    public final BooleanSupplier isEncoderConnected = fields.addBoolean("isEncoderConnected", this::getIsEncoderConnected);


    public ElevatorIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Outputs:
    
    public abstract double getHeightMeters();
    
    public abstract double getRightMotorCurrent();

    public abstract double getLeftMotorCurrent();

    protected abstract boolean getIsEncoderConnected();

    // Inputs:

    public abstract void setVoltage(double voltage);

}
