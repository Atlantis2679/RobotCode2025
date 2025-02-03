package frc.robot.subsystems.pivot.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.RobotMap.CANBUS;
import static frc.robot.subsystems.pivot.PivotConstants.*;

public class PivotIOSparxmax extends PivotIO{
        private final SparkMax pivotLeftMotor = new SparkMax(CANBUS.PIVOT_LEFT_MOTOR_ID, MotorType.kBrushless);
        private final SparkMax pivotRightMotor = new SparkMax(CANBUS.PIVOT_RIGHT_MOTOR_ID, MotorType.kBrushless);
        private final DutyCycleEncoder encoder = new DutyCycleEncoder(CANBUS.PIVOT_ENCODER_ID);
        private final SparkMaxConfig config = new SparkMaxConfig();
        public PivotIOSparxmax(LogFieldsTable fieldsTable) {
            super(fieldsTable);
            config.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
            pivotLeftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            pivotRightMotor.configure(config.inverted(true), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        //outputs
        @Override
        protected double getLeftMotorCurrent() {
            return pivotLeftMotor.getOutputCurrent();
        }

        @Override
        protected double getRightMotorCurrent() {
            return pivotRightMotor.getOutputCurrent();
        }

        @Override
        protected double getPivotAngleDegrees() {
            return encoder.get();
        }

        //inputs
        @Override
        public void setVoltage(double voltage) {
            pivotLeftMotor.set(voltage);
            pivotRightMotor.set(voltage);
        }

}