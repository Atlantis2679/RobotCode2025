package frc.robot.subsystems.funnel.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class FunnelIOSparkMax extends FunnelIO{
    
    private DigitalInput beambBreak;
    private SparkMax motor;
    public FunnelIOSparkMax(LogFieldsTable logFieldsTable){
        super(logFieldsTable);
        this.beambBreak = new DigitalInput(1);
        this.motor = new SparkMax(10, MotorType.kBrushless);
    }
    public void setVoltage(double voltage){
        this.motor.set(voltage);
    }
    protected boolean getBeamBreak(){
        return this.beambBreak.get();
    }
}