package frc.robot.subsystems.gripper.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.*;

public class GripperIOSparkMax extends GripperIO {
    private final SparkMax rightMotor = new SparkMax(CANBUS.GRIPPER_RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(CANBUS.GRIPPER_LEFT_MOTOR_ID, MotorType.kBrushless);

    private final DigitalInput beamBrake = new DigitalInput(GRIPPER_BEAM_BRAKE_ID); 

    public GripperIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Inputs:

    @Override
    protected boolean getIsCoralIn() {
        return beamBrake.get();
    }

    // Outputs:

    @Override
    public void setRightMotorVoltage(double voltage) {
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void setLeftMotorVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }
}
