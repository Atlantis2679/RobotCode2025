package frc.robot.subsystems.gripper.io;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.*;

public class GripperIOSparkMax extends GripperIO {
    private final SparkMax rightOutTakeMotor = new SparkMax(GRIPPER_RIGHT_OUTTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftOutTakeMotor = new SparkMax(GRIPPER_LEFT_OUTTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax backMotor = new SparkMax(GRIPPER_IBACK_MOTOR_ID, MotorType.kBrushless);

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
    public void setRightOutTakeMotorVoltage(double voltage) {
        rightOutTakeMotor.setVoltage(voltage);
    }

    @Override
    public void setRightOutTakeMotorPrecentageSpeed(double precentageSpeed) {
        rightOutTakeMotor.set(precentageSpeed);
    }

    @Override
    public void setLeftOutTakeMotorVoltage(double voltage) {
        leftOutTakeMotor.setVoltage(voltage);
    }

    @Override
    public void setLeftOutTakeMotorPrecentageSpeed(double precentageSpeed) {
        leftOutTakeMotor.set(precentageSpeed);
    }

    @Override
    public void setBackMotorVoltage(double voltage) {
        backMotor.setVoltage(voltage);
    }

    @Override
    public void setBackMotorPrecentageSpeed(double precentageSpeed) {
        backMotor.set(precentageSpeed);
    }
}
