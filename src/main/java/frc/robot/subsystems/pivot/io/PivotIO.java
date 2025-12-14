package frc.robot.subsystems.pivot.io;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class PivotIO extends IOBase {
    
    //constructor:
    public PivotIO(LogFieldsTable logFieldsTable){
        super(logFieldsTable);
    }

    //output methods:
    public abstract double getMotorVolt();
    public abstract double getMotorAngle();
    public abstract boolean isEncoderConnected();

    //input methods:
    public abstract void setMotorVolt();

}