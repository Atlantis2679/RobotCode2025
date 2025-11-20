package frc.robot.subsystems.pivot.io;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.pivot.PivotConstants;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class PivotIOSim extends PivotIO{

    public PivotIOSim(LogFieldsTable fields){
        super(fields);
    }

    private SingleJointedArmSim motor = new SingleJointedArmSim(DCMotor.getNEO(1), 
        PivotConstants.Sim.JOINT_GEAR_RATIO,
        PivotConstants.Sim.JKG_METERS_SQUARED, 
        PivotConstants.Sim.ARM_LENGTH_M, 
        Math.toRadians(PivotConstants.Sim.TURNING_MIN_DEGREES_DEG), 
        Math.toRadians(PivotConstants.Sim.TURNING_MAX_DEGREES_DEG), 
        true, 
        Math.toRadians(PivotConstants.ANGLE_OFFSET));

    @Override
    protected void periodicBeforeFields() {
        motor.update(0.02);
    }

    @Override
    public boolean isEncoderConnected(){
        return true;
    }

    @Override
    public double getMotorCurrent(){
        return motor.getCurrentDrawAmps();
    }

    @Override
    public double getPivotAngleDegrees(){
        return Math.toDegrees(motor.getAngleRads());
    }


    @Override
    public void setVoltage(double voltage){
        motor.setInputVoltage(voltage);
    }

}
